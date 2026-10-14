// =============================================================================
// 模块: sys_route_control_core
// 作用: 未被当前顶层采用的另一版路线核心，保留在 Robei 项目清单中。
// 说明: 它直接在 64 bit route_info 上移位，并使用 step_start/step_words 保存撤销边界。
//
// Robei 源: little_car/model/sys_route_control_core.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module sys_route_control_core (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire key_up_pulse,  // 消抖后的上键单周期按下事件；编辑态用于添加前进，菜单态用于切换焦点。
  input wire key_down_pulse,  // 消抖后的下键单周期按下事件；编辑态用于撤销最后一个 UI 动作。
  input wire key_left_pulse,  // 消抖后的左键单周期按下事件；编辑态写入左转加前进复合动作。
  input wire key_right_pulse,  // 消抖后的右键单周期按下事件；编辑态写入右转加前进复合动作。
  input wire key_mid_pulse,  // 消抖后的中键单周期按下事件；用于确认菜单、开始运行或从停车返回。
  output reg [63:0] old_route_info,  // 已完成路线历史，用于 GUI 和手机显示。
  output wire [15:0] complete_route_length,  // completed_route_steps 乘以 step_length 得到的已完成路线长度。
  input wire run_step_pulse,  // 一条运动动作完成的单周期脉冲。
  input wire obstacle_stop,  // 合并或当前模块产生的停车请求。
  output reg [1:0] menu_info,  // 菜单焦点/页面状态编码。
  output reg [4:0] state_info,  // 路线 FSM 状态编码。
  output reg [63:0] route_info,  // 64 bit 路线命令队列；最低 2 bit 是下一条命令。
  output reg [5:0] active_route_steps,  // 当前待执行内部命令数量。
  output reg [5:0] completed_route_steps,  // 已完成内部命令数量。
  output wire [15:0] step_length,  // 当前生效的路线单步长度；设置为零时路线核心回退到默认值 20。
  output wire [15:0] speed,  // 当前生效的路线整体速度参数，向 GUI 和执行控制链同步。
  input wire [15:0] step_length_set,  // 手机设置的单步长度参数。
  input wire [15:0] speed_set,  // 路线控制采用的整体速度参数。
  output wire [15:0] total_route_length,  // 已完成步数与待执行步数之和乘以 step_length 得到的计划总长度。
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

  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  reg focus_run;  // 旧版菜单焦点：0 选择路线编辑，1 选择开始运行。
  reg [1:0] now_dir;  // 旧版路线核心记录的小车当前朝向，采用 DIR_UP/RIGHT/DOWN/LEFT 模 4 编码。
  reg [1:0] target_dir;  // 旧版路线队列全部执行后预测的小车朝向；编辑和撤销时重新计算。

  reg [5:0] step_count;  // 旧版用户动作数量；用于索引 step_start/step_words 撤销元数据。
  reg [5:0] step_start [0:31];  // 32 项 UI 动作各自在 64 bit route_info 中的起始命令槽下标。
  reg [1:0] step_words [0:31];  // 32 项 UI 动作各占用的内部命令数；前进为 1，转向加前进为 2。

  integer i;  // 旧版路线数组循环下标；用于清零 32 项撤销元数据、重放有效命令重建目标位置，以及撤销时定位需要清除的命令槽。
  reg [5:0] undo_start;  // 组合读取的最后一个 UI 动作起始命令槽，用于一次撤销整组命令。
  reg [1:0] undo_words;  // 组合读取的最后一个 UI 动作命令数量，决定撤销时清除一个还是两个槽。
  reg [1:0] run_cmd;  // 旧版自动运行时从 route_info[1:0] 读取的当前队首命令。

  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“step_length”：当前生效的路线单步长度；设置为零时路线核心回退到默认值 20。 该组合结果直接由 `step_length_set`（步骤 / 长度 / 设定值）的当前值决定，不引入时钟周期延迟。
  assign step_length = (step_length_set == 16'd0) ? 16'd20 : step_length_set;
  // [ASSIGN] 组合定义“speed”：当前生效的路线整体速度参数，向 GUI 和执行控制链同步。 该组合结果直接由 `speed_set`（速度 / 设定值）的当前值决定，不引入时钟周期延迟。
  assign speed = speed_set;
  // [ASSIGN] 组合定义“total_route_length”：已完成步数与待执行步数之和乘以 step_length 得到的计划总长度。 该组合结果直接由 `completed_route_steps`（已完成 / 路线 / 步数）、`active_route_steps`（当前有效 / 路线 / 步数）、`step_length`（步骤 / 长度）的当前值决定，不引入时钟周期延迟。
  assign total_route_length = (completed_route_steps + active_route_steps) * step_length;
  // [ASSIGN] 组合定义“complete_route_length”：completed_route_steps 乘以 step_length 得到的已完成路线长度。 该组合结果直接由 `completed_route_steps`（已完成 / 路线 / 步数）、`step_length`（步骤 / 长度）的当前值决定，不引入时钟周期延迟。
  assign complete_route_length = completed_route_steps * step_length;
  // [ASSIGN] 组合定义“remaining_route_length”：active_route_steps 乘以 step_length 得到的剩余路线长度。 该组合结果直接由 `active_route_steps`（当前有效 / 路线 / 步数）、`step_length`（步骤 / 长度）的当前值决定，不引入时钟周期延迟。
  assign remaining_route_length = active_route_steps * step_length;

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // 组合映射 menu_info：根据 state_info 和 focus_run 选择菜单焦点/进入状态编码。
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
  // [TASK] 过程任务: apply_route_cmd_to_target
  // 把一条 2 bit 路线命令应用到预测终点和 target_dir；转向只改朝向，前进只改坐标。
  // -----------------------------------------------------------------------------
  task apply_route_cmd_to_target;
    input [1:0] cmd;  // 2 bit 任务 `apply_route_cmd_to_target` 的“命令”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      if (cmd == ROUTE_RIGHT)
        target_dir = target_dir + 2'd1;
      else if (cmd == ROUTE_LEFT)
        target_dir = target_dir - 2'd1;
      else if (cmd == ROUTE_FORWARD) begin
        case (target_dir)
          DIR_UP:    target_y = target_y - 8'd1;
          DIR_RIGHT: target_x = target_x + 8'd1;
          DIR_DOWN:  target_y = target_y + 8'd1;
          default:   target_x = target_x - 8'd1;
        endcase
      end
    end
  endtask

  // -----------------------------------------------------------------------------
  // [TASK] 过程任务: apply_route_cmd_to_now
  // 把一条已完成命令应用到当前坐标和 now_dir，重建小车实际逻辑位置。
  // -----------------------------------------------------------------------------
  task apply_route_cmd_to_now;
    input [1:0] cmd;  // 2 bit 任务 `apply_route_cmd_to_now` 的“命令”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      if (cmd == ROUTE_RIGHT)
        now_dir = now_dir + 2'd1;
      else if (cmd == ROUTE_LEFT)
        now_dir = now_dir - 2'd1;
      else if (cmd == ROUTE_FORWARD) begin
        case (now_dir)
          DIR_UP:    now_y = now_y - 8'd1;
          DIR_RIGHT: now_x = now_x + 8'd1;
          DIR_DOWN:  now_y = now_y + 8'd1;
          default:   now_x = now_x - 8'd1;
        endcase
      end
    end
  endtask

  // -----------------------------------------------------------------------------
  // [TASK] 过程任务: clear_active_route
  // 清空旧版待执行队列及 32 项撤销元数据，并把预测终点重新对齐当前位置。
  // -----------------------------------------------------------------------------
  task clear_active_route;
    begin
      route_info <= 64'd0;
      active_route_steps <= 6'd0;
      step_count <= 6'd0;
      target_x <= now_x;
      target_y <= now_y;
      target_dir <= now_dir;
      for (i = 0; i < 32; i = i + 1) begin
        step_start[i] <= 6'd0;
        step_words[i] <= 2'd0;
      end
    end
  endtask

  // -----------------------------------------------------------------------------
  // [TASK] 过程任务: rebuild_target_from_active
  // 从当前位置起重放 route_info 前 limit_count 条命令，重新计算撤销后的预测终点和朝向。
  // -----------------------------------------------------------------------------
  task rebuild_target_from_active;
    input [5:0] limit_count;  // 6 bit 任务 `rebuild_target_from_active` 的“limit / 计数”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
    begin
      target_x = now_x;
      target_y = now_y;
      target_dir = now_dir;
      for (i = 0; i < 32; i = i + 1) begin
        if (i < limit_count) begin
          if (route_info[i*2 +: 2] != ROUTE_STOP)
            apply_route_cmd_to_target(route_info[i*2 +: 2]);
        end
      end
    end
  endtask

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 2
  // 旧版路线主状态机：直接移位 64 bit route_info，维护撤销边界、历史路线、坐标和运行出队。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      focus_run <= 1'b0;
      state_info <= STATE_MENU;
      route_info <= 64'd0;
      old_route_info <= 64'd0;
      active_route_steps <= 6'd0;
      completed_route_steps <= 6'd0;
      step_count <= 6'd0;
      now_x <= 8'd128;
      now_y <= 8'd128;
      target_x <= 8'd128;
      target_y <= 8'd128;
      now_dir <= DIR_UP;
      target_dir <= DIR_UP;

      for (i = 0; i < 32; i = i + 1) begin
        step_start[i] <= 6'd0;
        step_words[i] <= 2'd0;
      end
    end else begin
      case (state_info)
        STATE_CHANGE: begin
          if (key_mid_pulse) begin
            state_info <= STATE_MENU;
          end else if (key_down_pulse) begin
            if (step_count != 6'd0) begin
              undo_start = step_start[step_count - 6'd1];
              undo_words = step_words[step_count - 6'd1];

              for (i = 0; i < 32; i = i + 1) begin
                if ((i >= undo_start) && (i < undo_start + undo_words))
                  route_info[i*2 +: 2] <= 2'b00;
              end

              active_route_steps <= undo_start;
              step_count <= step_count - 6'd1;
              rebuild_target_from_active(undo_start);
            end
          end else if (key_up_pulse) begin
            if (active_route_steps < 6'd32 && step_count < 6'd32) begin
              route_info[active_route_steps*2 +: 2] <= ROUTE_FORWARD;
              step_start[step_count] <= active_route_steps;
              step_words[step_count] <= 2'd1;
              active_route_steps <= active_route_steps + 6'd1;
              step_count <= step_count + 6'd1;
              apply_route_cmd_to_target(ROUTE_FORWARD);
            end
          end else if (key_left_pulse) begin
            if (active_route_steps < 6'd31 && step_count < 6'd32) begin
              route_info[active_route_steps*2 +: 2] <= ROUTE_LEFT;
              route_info[(active_route_steps + 6'd1)*2 +: 2] <= ROUTE_FORWARD;
              step_start[step_count] <= active_route_steps;
              step_words[step_count] <= 2'd2;
              active_route_steps <= active_route_steps + 6'd2;
              step_count <= step_count + 6'd1;
              apply_route_cmd_to_target(ROUTE_LEFT);
              apply_route_cmd_to_target(ROUTE_FORWARD);
            end
          end else if (key_right_pulse) begin
            if (active_route_steps < 6'd31 && step_count < 6'd32) begin
              route_info[active_route_steps*2 +: 2] <= ROUTE_RIGHT;
              route_info[(active_route_steps + 6'd1)*2 +: 2] <= ROUTE_FORWARD;
              step_start[step_count] <= active_route_steps;
              step_words[step_count] <= 2'd2;
              active_route_steps <= active_route_steps + 6'd2;
              step_count <= step_count + 6'd1;
              apply_route_cmd_to_target(ROUTE_RIGHT);
              apply_route_cmd_to_target(ROUTE_FORWARD);
            end
          end
        end

        STATE_RUN: begin
          if (obstacle_stop) begin
            state_info <= STATE_STOP;
            clear_active_route();
          end else if (run_step_pulse && active_route_steps != 6'd0) begin
            run_cmd = route_info[1:0];
            old_route_info[completed_route_steps*2 +: 2] <= run_cmd;
            completed_route_steps <= completed_route_steps + 6'd1;
            apply_route_cmd_to_now(run_cmd);
            route_info <= {2'b00, route_info[63:2]};
            active_route_steps <= active_route_steps - 6'd1;
          end else if (active_route_steps == 6'd0) begin
            state_info <= STATE_MENU;
            focus_run <= 1'b0;
          end
        end

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
