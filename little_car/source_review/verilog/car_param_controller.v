// =============================================================================
// 模块: car_param_controller
// 作用: 保存并限幅所有手机可调运行参数。
// 说明: 参数包括整体速度、单步长度、四轮默认速度和雷达安全距离。
// 说明: 速度范围 100..900、步长 5..80、雷达阈值 150..1500 mm，所有修改均饱和限幅。
// 说明: selected_param 编码 0..6；配置脉冲来自 bt_route_uart。
//
// Robei 源: little_car/model/car_param_controller.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module car_param_controller (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire key_up_n,  // 低有效按键输入/输出；0 表示按下。
  input wire key_down_n,  // 低有效按键输入/输出；0 表示按下。
  input wire key_left_n,  // 低有效按键输入/输出；0 表示按下。
  input wire key_right_n,  // 低有效按键输入/输出；0 表示按下。
  input wire key_mid_n,  // 低有效按键输入/输出；0 表示按下。
  input wire bt_cfg_mode_pulse,  // 手机切换参数设置模式的单周期事件脉冲。
  input wire bt_cfg_next_pulse,  // 手机选择下一项可调参数的单周期事件脉冲。
  input wire bt_cfg_prev_pulse,  // 手机选择上一项可调参数的单周期事件脉冲。
  input wire bt_cfg_inc_pulse,  // 手机把当前所选参数增加一个固定步长的单周期事件脉冲。
  input wire bt_cfg_dec_pulse,  // 手机把当前所选参数减少一个固定步长的单周期事件脉冲。
  input wire [2:0] bt_cfg_select,  // 手机直接指定的参数编号 0..6；仅在 bt_cfg_select_valid 脉冲周期被参数控制器采纳。
  input wire bt_cfg_select_valid,  // bt_cfg_select 携带新参数编号的单周期有效脉冲。
  output reg route_key_lock,  // 参数设置期间的路线按键锁；为 1 时实体/蓝牙方向键不会修改路线。
  output reg setting_mode,  // 手机参数设置模式；为 1 时参数按键优先并锁住路线编辑按键。
  output reg [2:0] selected_param,  // 当前手机增减操作选中的参数编号 0..6。
  output reg [15:0] step_length_set,  // 手机设置的单步长度参数。
  output wire [15:0] speed_set,  // 路线控制采用的整体速度参数。
  output reg [9:0] overall_speed,  // 手机可调的公共速度基准。
  output reg [9:0] lf_default_speed,  // 左前轮校准默认速度。
  output reg [9:0] rf_default_speed,  // 右前轮校准默认速度。
  output reg [9:0] lr_default_speed,  // 左后轮校准默认速度。
  output reg [9:0] rr_default_speed,  // 右后轮校准默认速度。
  output reg [15:0] lidar_safe_mm  // 手机可调雷达正前方安全距离，单位 mm。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
        // -----------------------------------------------------------------------------
        // 组合派生信号
        // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
        // -----------------------------------------------------------------------------
        // [ASSIGN] 组合定义“speed_set”：路线控制采用的整体速度参数。 该组合结果直接由 `overall_speed`（整体 / 速度）的当前值决定，不引入时钟周期延迟。
        assign speed_set = {6'd0, overall_speed};

        // -----------------------------------------------------------------------------
        // 常量、状态编码与时序参数
        // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
        // -----------------------------------------------------------------------------
        localparam [2:0] PARAM_OVERALL = 3'd0;  // 可调参数“PARAM / OVERALL”的选择索引，供手机直接选中后执行增减。
        localparam [2:0] PARAM_STEP    = 3'd1;  // 可调参数“PARAM / STEP”的选择索引，供手机直接选中后执行增减。
        localparam [2:0] PARAM_LF      = 3'd2;  // 可调参数“PARAM / LF”的选择索引，供手机直接选中后执行增减。
        localparam [2:0] PARAM_RF      = 3'd3;  // 可调参数“PARAM / RF”的选择索引，供手机直接选中后执行增减。
        localparam [2:0] PARAM_LR      = 3'd4;  // 可调参数“PARAM / LR”的选择索引，供手机直接选中后执行增减。
        localparam [2:0] PARAM_RR      = 3'd5;  // 可调参数“PARAM / RR”的选择索引，供手机直接选中后执行增减。
        localparam [2:0] PARAM_LIDAR   = 3'd6;  // 可调参数“PARAM / LIDAR”的选择索引，供手机直接选中后执行增减。

        localparam [9:0] SPEED_MIN = 10'd100;  // “SPEED / MIN”允许的饱和下限；继续减小时保持此值。
        localparam [9:0] SPEED_MAX = 10'd900;  // “SPEED / MAX”允许的饱和上限；继续增加时保持此值。
        localparam [9:0] SPEED_STEP = 10'd25;  // 每次手机增减命令对“SPEED / STEP”施加的固定步进。
        localparam [15:0] STEP_MIN = 16'd5;  // “STEP / MIN”允许的饱和下限；继续减小时保持此值。
        localparam [15:0] STEP_MAX = 16'd80;  // “STEP / MAX”允许的饱和上限；继续增加时保持此值。
        localparam [15:0] STEP_STEP = 16'd5;  // 每次手机增减命令对“STEP / STEP”施加的固定步进。
        localparam [15:0] LIDAR_SAFE_MIN = 16'd150;  // “LIDAR / SAFE / MIN”允许的饱和下限；继续减小时保持此值。
        localparam [15:0] LIDAR_SAFE_MAX = 16'd1500;  // “LIDAR / SAFE / MAX”允许的饱和上限；继续增加时保持此值。
        localparam [15:0] LIDAR_SAFE_STEP = 16'd50;  // 每次手机增减命令对“LIDAR / SAFE / STEP”施加的固定步进。

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: next_param
        // 在整体速度、步长、四轮默认速度和雷达阈值七项之间循环选择下一项。
        // -----------------------------------------------------------------------------
        function [2:0] next_param;
          input [2:0] value;  // 3 bit 函数 `next_param` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          begin
            next_param = (value >= PARAM_LIDAR) ? PARAM_OVERALL : (value + 3'd1);
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: prev_param
        // 在七项手机可调参数之间反向循环选择，PARAM_OVERALL 的前一项回绕到 PARAM_LIDAR。
        // -----------------------------------------------------------------------------
        function [2:0] prev_param;
          input [2:0] value;  // 3 bit 函数 `prev_param` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          begin
            prev_param = (value == PARAM_OVERALL) ? PARAM_LIDAR : (value - 3'd1);
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: inc_speed
        // 把速度增加 25，并在 900 上限处饱和，避免参数溢出。
        // -----------------------------------------------------------------------------
        function [9:0] inc_speed;
          input [9:0] value;  // 10 bit 函数 `inc_speed` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          begin
            inc_speed = (value >= SPEED_MAX - SPEED_STEP) ? SPEED_MAX :
                        (value + SPEED_STEP);
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: dec_speed
        // 把速度减少 25，并在 100 下限处饱和，避免参数下溢。
        // -----------------------------------------------------------------------------
        function [9:0] dec_speed;
          input [9:0] value;  // 10 bit 函数 `dec_speed` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          begin
            dec_speed = (value <= SPEED_MIN + SPEED_STEP) ? SPEED_MIN :
                        (value - SPEED_STEP);
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: inc_step
        // 把路线单步长度增加 5，并在 80 上限处饱和。
        // -----------------------------------------------------------------------------
        function [15:0] inc_step;
          input [15:0] value;  // 16 bit 函数 `inc_step` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          begin
            inc_step = (value >= STEP_MAX - STEP_STEP) ? STEP_MAX :
                       (value + STEP_STEP);
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: dec_step
        // 把路线单步长度减少 5，并在 5 下限处饱和。
        // -----------------------------------------------------------------------------
        function [15:0] dec_step;
          input [15:0] value;  // 16 bit 函数 `dec_step` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          begin
            dec_step = (value <= STEP_MIN + STEP_STEP) ? STEP_MIN :
                       (value - STEP_STEP);
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: inc_lidar_safe
        // 把雷达正前方安全距离增加 50 mm，并在 1500 mm 上限处饱和。
        // -----------------------------------------------------------------------------
        function [15:0] inc_lidar_safe;
          input [15:0] value;  // 16 bit 函数 `inc_lidar_safe` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          begin
            inc_lidar_safe = (value >= LIDAR_SAFE_MAX - LIDAR_SAFE_STEP) ? LIDAR_SAFE_MAX :
                             (value + LIDAR_SAFE_STEP);
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [FUNCTION] 辅助函数: dec_lidar_safe
        // 把雷达正前方安全距离减少 50 mm，并在 150 mm 下限处饱和。
        // -----------------------------------------------------------------------------
        function [15:0] dec_lidar_safe;
          input [15:0] value;  // 16 bit 函数 `dec_lidar_safe` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
          begin
            dec_lidar_safe = (value <= LIDAR_SAFE_MIN + LIDAR_SAFE_STEP) ? LIDAR_SAFE_MIN :
                             (value - LIDAR_SAFE_STEP);
          end
        endfunction

        // -----------------------------------------------------------------------------
        // [PROCESS] 过程块 1
        // 参数寄存器：处理模式切换、参数选择、增减脉冲和饱和上下限。
        // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
        // -----------------------------------------------------------------------------
        always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
            route_key_lock <= 1'b0;
            setting_mode <= 1'b0;
            selected_param <= PARAM_OVERALL;
            step_length_set <= 16'd20;
            overall_speed <= 10'd700;
            lf_default_speed <= 10'd700;
            rf_default_speed <= 10'd700;
            lr_default_speed <= 10'd700;
            rr_default_speed <= 10'd700;
            lidar_safe_mm <= 16'd450;
          end else begin
            route_key_lock <= 1'b0;

            if (bt_cfg_mode_pulse)
              setting_mode <= ~setting_mode;

            if (bt_cfg_select_valid) begin
              setting_mode <= 1'b1;
              if (bt_cfg_select <= PARAM_LIDAR)
                selected_param <= bt_cfg_select;
            end

            if (bt_cfg_next_pulse) begin
              setting_mode <= 1'b1;
              selected_param <= next_param(selected_param);
            end

            if (bt_cfg_prev_pulse) begin
              setting_mode <= 1'b1;
              selected_param <= prev_param(selected_param);
            end

            if (bt_cfg_inc_pulse) begin
              setting_mode <= 1'b1;
              case (selected_param)
                PARAM_STEP: step_length_set <= inc_step(step_length_set);
                PARAM_LF: lf_default_speed <= inc_speed(lf_default_speed);
                PARAM_RF: rf_default_speed <= inc_speed(rf_default_speed);
                PARAM_LR: lr_default_speed <= inc_speed(lr_default_speed);
                PARAM_RR: rr_default_speed <= inc_speed(rr_default_speed);
                PARAM_LIDAR: lidar_safe_mm <= inc_lidar_safe(lidar_safe_mm);
                default: overall_speed <= inc_speed(overall_speed);
              endcase
            end

            if (bt_cfg_dec_pulse) begin
              setting_mode <= 1'b1;
              case (selected_param)
                PARAM_STEP: step_length_set <= dec_step(step_length_set);
                PARAM_LF: lf_default_speed <= dec_speed(lf_default_speed);
                PARAM_RF: rf_default_speed <= dec_speed(rf_default_speed);
                PARAM_LR: lr_default_speed <= dec_speed(lr_default_speed);
                PARAM_RR: rr_default_speed <= dec_speed(rr_default_speed);
                PARAM_LIDAR: lidar_safe_mm <= dec_lidar_safe(lidar_safe_mm);
                default: overall_speed <= dec_speed(overall_speed);
              endcase
            end
          end
        end

endmodule
