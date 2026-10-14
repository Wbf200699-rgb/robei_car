// =============================================================================
// 模块: route_contol_core
// 作用: 当前实际使用的路线编辑、执行、坐标更新状态机。
// 说明: route_info 每 2 bit 保存一条内部命令：00 停止、01 右转、10 左转、11 前进。
// 说明: 编辑时一次左/右操作会写入转向和前进两个内部命令，因此 UI 步和内部命令数不同。
// 说明: 运行时 c5_route_controller 的 run_step_pulse 使队首命令出队并更新当前位置/朝向。
// 说明: 重要风险：数组 ROUTE_WORDS=16，但前进分支的部分边界判断允许计数小于 32。
//
// Robei 源: little_car/model/route_contol_core.model
// 导出范围: 顶层实际依赖模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module route_contol_core (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire key_up_pulse,  // 消抖后的上键单周期按下事件；编辑态用于添加前进，菜单态用于切换焦点。
  input wire key_down_pulse,  // 消抖后的下键单周期按下事件；编辑态用于撤销最后一个 UI 动作。
  input wire key_left_pulse,  // 消抖后的左键单周期按下事件；编辑态写入左转加前进复合动作。
  input wire key_right_pulse,  // 消抖后的右键单周期按下事件；编辑态写入右转加前进复合动作。
  input wire key_mid_pulse,  // 消抖后的中键单周期按下事件；用于确认菜单、开始运行或从停车返回。
  output reg [1:0] menu_info,  // 菜单焦点/页面状态编码。
  output reg [63:0] route_info,  // 64 bit 路线命令队列；最低 2 bit 是下一条命令。
  input wire run_step_pulse,  // 一条运动动作完成的单周期脉冲。
  input wire obstacle_stop,  // 合并或当前模块产生的停车请求。
  input wire [15:0] step_length_set,  // 手机设置的单步长度参数。
  input wire [15:0] speed_set,  // 路线控制采用的整体速度参数。
  output reg [4:0] state_info,  // 路线 FSM 状态编码。
  output reg [63:0] old_route_info,  // 已完成路线历史，用于 GUI 和手机显示。
  output reg [5:0] active_route_steps,  // 当前待执行内部命令数量。
  output reg [5:0] completed_route_steps,  // 已完成内部命令数量。
  output wire [15:0] step_length,  // 当前生效的路线单步长度；设置为零时路线核心回退到默认值 20。
  output wire [15:0] speed,  // 当前生效的路线整体速度参数，向 GUI 和执行控制链同步。
  output wire [15:0] total_route_length,  // 已完成步数与待执行步数之和乘以 step_length 得到的计划总长度。
  output wire [15:0] complete_route_length,  // completed_route_steps 乘以 step_length 得到的已完成路线长度。
  output wire [15:0] remaining_route_length,  // active_route_steps 乘以 step_length 得到的剩余路线长度。
  output reg [7:0] now_x,  // 小车当前逻辑栅格 X 坐标；自动动作完成后由路线核心更新。
  output reg [7:0] now_y,  // 小车当前逻辑栅格 Y 坐标；自动动作完成后由路线核心更新。
  output reg [7:0] target_x,  // 编辑路线末端预测 X 坐标；添加或撤销动作时同步变化。
  output reg [7:0] target_y  // 编辑路线末端预测 Y 坐标；添加或撤销动作时同步变化。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 常量、状态编码与时序参数
  // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
  // -----------------------------------------------------------------------------
  localparam [4:0] STATE_MENU   = 5'b00000;  // 路线/业务状态“STATE / MENU”的固定编码；case 分支和状态上报必须保持一致。
  localparam [4:0] STATE_CHANGE = 5'b00001;  // 路线/业务状态“STATE / CHANGE”的固定编码；case 分支和状态上报必须保持一致。
  localparam [4:0] STATE_RUN    = 5'b00011;  // 路线/业务状态“STATE / RUN”的固定编码；case 分支和状态上报必须保持一致。
  localparam [4:0] STATE_STOP   = 5'b00100;  // 路线/业务状态“STATE / STOP”的固定编码；case 分支和状态上报必须保持一致。

  localparam [1:0] MENU_FOCUS_CHANGE = 2'b00;  // 菜单显示状态“MENU / FOCUS / CHANGE”的固定编码。
  localparam [1:0] MENU_IN_CHANGE    = 2'b01;  // 菜单显示状态“MENU / IN / CHANGE”的固定编码。
  localparam [1:0] MENU_FOCUS_RUN    = 2'b10;  // 菜单显示状态“MENU / FOCUS / RUN”的固定编码。
  localparam [1:0] MENU_IN_RUN       = 2'b11;  // 菜单显示状态“MENU / IN / RUN”的固定编码。

  localparam [1:0] ROUTE_STOP    = 2'b00;  // 2 bit 路线命令“ROUTE / STOP”的编码，存入 route_info 对应槽位。
  localparam [1:0] ROUTE_RIGHT   = 2'b01;  // 2 bit 路线命令“ROUTE / RIGHT”的编码，存入 route_info 对应槽位。
  localparam [1:0] ROUTE_LEFT    = 2'b10;  // 2 bit 路线命令“ROUTE / LEFT”的编码，存入 route_info 对应槽位。
  localparam [1:0] ROUTE_FORWARD = 2'b11;  // 2 bit 路线命令“ROUTE / FORWARD”的编码，存入 route_info 对应槽位。

  localparam [1:0] DIR_UP    = 2'd0;  // 小车朝向“DIR / UP”的模 4 编码；加减 1 分别完成 90 度右/左转。
  localparam [1:0] DIR_RIGHT = 2'd1;  // 小车朝向“DIR / RIGHT”的模 4 编码；加减 1 分别完成 90 度右/左转。
  localparam [1:0] DIR_DOWN  = 2'd2;  // 小车朝向“DIR / DOWN”的模 4 编码；加减 1 分别完成 90 度右/左转。
  localparam [1:0] DIR_LEFT  = 2'd3;  // 小车朝向“DIR / LEFT”的模 4 编码；加减 1 分别完成 90 度右/左转。
  localparam integer ROUTE_WORDS = 16;  // 本模块实际展开/显示的 2 bit 路线命令槽数量；数组和循环边界均以此为准。
  localparam integer ROUTE_LAST = ROUTE_WORDS - 1;  // “ROUTE / LAST”数组或帧的最后一个有效零基下标。

  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  integer i;  // 路线数组循环下标；组合打包 route_info/old_route_info、复位清零队列以及运行出队左移时遍历 0..ROUTE_WORDS-1。
  reg focus_run;  // 菜单焦点：0 选择编辑路线，1 选择开始运行。
  reg [1:0] now_dir;  // 小车当前朝向，采用 DIR_UP/RIGHT/DOWN/LEFT 编码。
  reg [1:0] target_dir;  // 编辑路线末端的预测朝向。
  reg [5:0] step_count;  // 用户编辑动作数；转向+前进虽然占两个命令，但只算一个 UI 动作。
  reg [15:0] step_is_turn;  // 每个 UI 动作是否为转向复合动作，用于一次撤销两个命令。
  reg [1:0] route_cmd_mem [0:ROUTE_WORDS-1];  // 待执行命令数组；索引 0 是自动运行的队首。
  reg [1:0] old_route_cmd_mem [0:ROUTE_WORDS-1];  // 已经执行完成的命令历史，用于屏幕显示走过的路径。

  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“step_length”：当前生效的路线单步长度；设置为零时路线核心回退到默认值 20。 该组合结果直接由 `step_length_set`（步骤 / 长度 / 设定值）的当前值决定，不引入时钟周期延迟。
  assign step_length = (step_length_set == 16'd0) ? 16'd20 : step_length_set;
  // [ASSIGN] 组合定义“speed”：当前生效的路线整体速度参数，向 GUI 和执行控制链同步。 该组合结果直接由 `speed_set`（速度 / 设定值）的当前值决定，不引入时钟周期延迟。
  assign speed = speed_set;
  // [ASSIGN] 组合定义“total_route_length”：已完成步数与待执行步数之和乘以 step_length 得到的计划总长度。 该组合结果直接由 `completed_route_steps`（已完成 / 路线 / 步数）、`active_route_steps`（当前有效 / 路线 / 步数）、`step_length`（步骤 / 长度）的当前值决定，不引入时钟周期延迟。
  assign total_route_length = {10'd0, completed_route_steps + active_route_steps} * step_length;
  // [ASSIGN] 组合定义“complete_route_length”：completed_route_steps 乘以 step_length 得到的已完成路线长度。 该组合结果直接由 `completed_route_steps`（已完成 / 路线 / 步数）、`step_length`（步骤 / 长度）的当前值决定，不引入时钟周期延迟。
  assign complete_route_length = {10'd0, completed_route_steps} * step_length;
  // [ASSIGN] 组合定义“remaining_route_length”：active_route_steps 乘以 step_length 得到的剩余路线长度。 该组合结果直接由 `active_route_steps`（当前有效 / 路线 / 步数）、`step_length`（步骤 / 长度）的当前值决定，不引入时钟周期延迟。
  assign remaining_route_length = {10'd0, active_route_steps} * step_length;

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // 组合映射 menu_info：把内部 FSM 状态和菜单焦点转换成 GUI 菜单编码。
  // -----------------------------------------------------------------------------
  always @(*) begin
    case (state_info)
      STATE_CHANGE: menu_info = MENU_IN_CHANGE;
      STATE_RUN:    menu_info = MENU_IN_RUN;
      STATE_STOP:   menu_info = MENU_FOCUS_CHANGE;
      default:      menu_info = focus_run ? MENU_FOCUS_RUN : MENU_FOCUS_CHANGE;
    endcase
  end

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 2
  // 组合打包路线数组：数组下标 i 映射到 route_info[i*2 +: 2]。
  // -----------------------------------------------------------------------------
  always @(*) begin
    route_info = 64'd0;
    old_route_info = 64'd0;
    for (i = 0; i < ROUTE_WORDS; i = i + 1) begin
      route_info[i*2 +: 2] = route_cmd_mem[i];
      old_route_info[i*2 +: 2] = old_route_cmd_mem[i];
    end
  end

  // -----------------------------------------------------------------------------
  // [TASK] 过程任务: move_target_forward
  // 按 target_dir 将编辑终点前移一个逻辑栅格。
  // -----------------------------------------------------------------------------
  task move_target_forward;
    begin
      case (target_dir)
        DIR_UP:    target_y <= target_y - 8'd1;
        DIR_RIGHT: target_x <= target_x + 8'd1;
        DIR_DOWN:  target_y <= target_y + 8'd1;
        default:   target_x <= target_x - 8'd1;
      endcase
    end
  endtask

  // -----------------------------------------------------------------------------
  // [TASK] 过程任务: move_target_backward
  // 撤销前进时将编辑终点后退一个逻辑栅格。
  // -----------------------------------------------------------------------------
  task move_target_backward;
    begin
      case (target_dir)
        DIR_UP:    target_y <= target_y + 8'd1;
        DIR_RIGHT: target_x <= target_x - 8'd1;
        DIR_DOWN:  target_y <= target_y - 8'd1;
        default:   target_x <= target_x + 8'd1;
      endcase
    end
  endtask

  // -----------------------------------------------------------------------------
  // [TASK] 过程任务: move_now_forward
  // 动作完成时按 now_dir 更新小车当前位置。
  // -----------------------------------------------------------------------------
  task move_now_forward;
    begin
      case (now_dir)
        DIR_UP:    now_y <= now_y - 8'd1;
        DIR_RIGHT: now_x <= now_x + 8'd1;
        DIR_DOWN:  now_y <= now_y + 8'd1;
        default:   now_x <= now_x - 8'd1;
      endcase
    end
  endtask

  // -----------------------------------------------------------------------------
  // [TASK] 过程任务: clear_active_route
  // 清空待执行路线，并把目标位置重新对齐当前位置。
  // -----------------------------------------------------------------------------
  task clear_active_route;
    begin
      for (i = 0; i < ROUTE_WORDS; i = i + 1)
        route_cmd_mem[i] <= ROUTE_STOP;
      active_route_steps <= 6'd0;
      step_count <= 6'd0;
      step_is_turn <= 16'd0;
      target_x <= now_x;
      target_y <= now_y;
      target_dir <= now_dir;
    end
  endtask

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 3
  // 路线主状态机：处理编辑、撤销、运行出队、坐标更新和障碍停止。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      focus_run <= 1'b0;
      state_info <= STATE_MENU;
      active_route_steps <= 6'd0;
      completed_route_steps <= 6'd0;
      step_count <= 6'd0;
      step_is_turn <= 32'd0;
      now_x <= 8'd128;
      now_y <= 8'd128;
      target_x <= 8'd128;
      target_y <= 8'd128;
      now_dir <= DIR_UP;
      target_dir <= DIR_UP;
      for (i = 0; i < ROUTE_WORDS; i = i + 1) begin
        route_cmd_mem[i] <= ROUTE_STOP;
        old_route_cmd_mem[i] <= ROUTE_STOP;
      end
    end else begin
      case (state_info)
        // -----------------------------------------------------------------------------
        // 关键分支
        // 编辑态：添加前进/转向复合动作，或按下键撤销最后一个 UI 动作。
        // -----------------------------------------------------------------------------
        STATE_CHANGE: begin
          if (key_mid_pulse) begin
            state_info <= STATE_MENU;
          end else if (key_down_pulse) begin
            if (step_count != 6'd0) begin
              if (step_is_turn[step_count - 6'd1]) begin
                route_cmd_mem[active_route_steps - 6'd1] <= ROUTE_STOP;
                route_cmd_mem[active_route_steps - 6'd2] <= ROUTE_STOP;
                active_route_steps <= active_route_steps - 6'd2;
                step_count <= step_count - 6'd1;
                step_is_turn[step_count - 6'd1] <= 1'b0;
                move_target_backward();
                if (route_cmd_mem[active_route_steps - 6'd2] == ROUTE_LEFT)
                  target_dir <= target_dir + 2'd1;
                else
                  target_dir <= target_dir - 2'd1;
              end else begin
                route_cmd_mem[active_route_steps - 6'd1] <= ROUTE_STOP;
                active_route_steps <= active_route_steps - 6'd1;
                step_count <= step_count - 6'd1;
                step_is_turn[step_count - 6'd1] <= 1'b0;
                move_target_backward();
              end
            end
          end else if (key_up_pulse) begin
            if (active_route_steps < 6'd32 && step_count < 6'd32) begin
              route_cmd_mem[active_route_steps] <= ROUTE_FORWARD;
              active_route_steps <= active_route_steps + 6'd1;
              step_count <= step_count + 6'd1;
              step_is_turn[step_count] <= 1'b0;
              move_target_forward();
            end
          end else if (key_left_pulse) begin
            if (active_route_steps < 6'd15 && step_count < 6'd16) begin
              route_cmd_mem[active_route_steps] <= ROUTE_LEFT;
              route_cmd_mem[active_route_steps + 6'd1] <= ROUTE_FORWARD;
              active_route_steps <= active_route_steps + 6'd2;
              step_count <= step_count + 6'd1;
              step_is_turn[step_count] <= 1'b1;
              target_dir <= target_dir - 2'd1;
              case (target_dir - 2'd1)
                DIR_UP:    target_y <= target_y - 8'd1;
                DIR_RIGHT: target_x <= target_x + 8'd1;
                DIR_DOWN:  target_y <= target_y + 8'd1;
                default:   target_x <= target_x - 8'd1;
              endcase
            end
          end else if (key_right_pulse) begin
            if (active_route_steps < 6'd15 && step_count < 6'd16) begin
              route_cmd_mem[active_route_steps] <= ROUTE_RIGHT;
              route_cmd_mem[active_route_steps + 6'd1] <= ROUTE_FORWARD;
              active_route_steps <= active_route_steps + 6'd2;
              step_count <= step_count + 6'd1;
              step_is_turn[step_count] <= 1'b1;
              target_dir <= target_dir + 2'd1;
              case (target_dir + 2'd1)
                DIR_UP:    target_y <= target_y - 8'd1;
                DIR_RIGHT: target_x <= target_x + 8'd1;
                DIR_DOWN:  target_y <= target_y + 8'd1;
                default:   target_x <= target_x - 8'd1;
              endcase
            end
          end
        end

        // -----------------------------------------------------------------------------
        // 关键分支
        // 运行态：障碍优先停止；否则每个 run_step_pulse 消费队首命令。
        // -----------------------------------------------------------------------------
        STATE_RUN: begin
          if (obstacle_stop) begin
            state_info <= STATE_STOP;
            clear_active_route();
          end else if (run_step_pulse && active_route_steps != 6'd0) begin
            old_route_cmd_mem[completed_route_steps] <= route_cmd_mem[0];
            completed_route_steps <= completed_route_steps + 6'd1;

            if (route_cmd_mem[0] == ROUTE_RIGHT)
              now_dir <= now_dir + 2'd1;
            else if (route_cmd_mem[0] == ROUTE_LEFT)
              now_dir <= now_dir - 2'd1;
            else
              move_now_forward();

            for (i = 0; i < ROUTE_LAST; i = i + 1)
              route_cmd_mem[i] <= route_cmd_mem[i + 1];
            route_cmd_mem[ROUTE_LAST] <= ROUTE_STOP;
            active_route_steps <= active_route_steps - 6'd1;
          end else if (active_route_steps == 6'd0) begin
            state_info <= STATE_MENU;
            focus_run <= 1'b0;
          end
        end

        // -----------------------------------------------------------------------------
        // 关键分支
        // 障碍停止态：等待中键返回编辑，并清除尚未执行的路线。
        // -----------------------------------------------------------------------------
        STATE_STOP: begin
          if (key_mid_pulse) begin
            state_info <= STATE_CHANGE;
            focus_run <= 1'b0;
            clear_active_route();
          end
        end

        default: begin
          state_info <= STATE_MENU;

          if (key_up_pulse)
            focus_run <= 1'b0;
          else if (key_down_pulse)
            focus_run <= 1'b1;

          if (key_mid_pulse) begin
            if (focus_run)
              state_info <= STATE_RUN;
            else
              state_info <= STATE_CHANGE;
          end
        end
      endcase
    end
  end

endmodule
