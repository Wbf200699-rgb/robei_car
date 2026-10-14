// =============================================================================
// 模块: gui_route_renderer
// 作用: 按像素坐标组合绘制小车状态、菜单、路线网格、当前位置和目标。
// 说明: 该模块不保存帧缓存，每个像素直接由坐标、路线和状态组合计算 RGB。
// 说明: 路线命令被展开为水平/垂直线段；字符使用 5x7 点阵放大两倍绘制。
// 说明: 这是工程组合逻辑和 LUT 占用最大的模块，改动字符/图形时需重点关注 ABC9 与布线。
//
// Robei 源: little_car/model/gui_route_renderer.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module gui_route_renderer (
  input wire [11:0] pixel_x,  // 当前有效显示区水平像素坐标。
  input wire [11:0] pixel_y,  // 当前有效显示区垂直像素坐标。
  input wire pixel_de,  // 视频数据有效；1 表示位于可见区域。
  input wire [1:0] menu_info,  // 菜单焦点/页面状态编码。
  input wire [63:0] route_info,  // 64 bit 路线命令队列；最低 2 bit 是下一条命令。
  input wire clk_pix,  // HDMI 像素时钟域时钟。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire [4:0] state_info,  // 路线 FSM 状态编码。
  input wire [63:0] old_route_info,  // 已完成路线历史，用于 GUI 和手机显示。
  input wire [5:0] active_route_steps,  // 当前待执行内部命令数量。
  input wire [5:0] completed_route_steps,  // 已完成内部命令数量。
  input wire [15:0] step_length,  // 当前生效的路线单步长度；设置为零时路线核心回退到默认值 20。
  input wire [15:0] speed,  // 当前生效的路线整体速度参数，向 GUI 和执行控制链同步。
  input wire [15:0] total_route_length,  // 已完成步数与待执行步数之和乘以 step_length 得到的计划总长度。
  input wire [15:0] complete_route_length,  // completed_route_steps 乘以 step_length 得到的已完成路线长度。
  input wire [15:0] remaining_route_length,  // active_route_steps 乘以 step_length 得到的剩余路线长度。
  input wire [7:0] now_x,  // 小车当前逻辑栅格 X 坐标；自动动作完成后由路线核心更新。
  input wire [7:0] now_y,  // 小车当前逻辑栅格 Y 坐标；自动动作完成后由路线核心更新。
  input wire [7:0] target_x,  // 编辑路线末端预测 X 坐标；添加或撤销动作时同步变化。
  input wire [7:0] target_y,  // 编辑路线末端预测 Y 坐标；添加或撤销动作时同步变化。
  input wire bt_connected,  // 仅在持续收到 P/p 心跳时为 1。
  output reg [23:0] rgb_data  // GUI 输出或 HDMI 输入的 RGB888 像素。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
          // -----------------------------------------------------------------------------
          // 常量、状态编码与时序参数
          // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
          // -----------------------------------------------------------------------------
          localparam [1:0] MENU_FOCUS_CHANGE = 2'b00;  // 菜单显示状态“MENU / FOCUS / CHANGE”的固定编码。
          localparam [1:0] MENU_IN_CHANGE    = 2'b01;  // 菜单显示状态“MENU / IN / CHANGE”的固定编码。
          localparam [1:0] MENU_FOCUS_RUN    = 2'b10;  // 菜单显示状态“MENU / FOCUS / RUN”的固定编码。
          localparam [1:0] MENU_IN_RUN       = 2'b11;  // 菜单显示状态“MENU / IN / RUN”的固定编码。

          localparam [4:0] STATE_MENU   = 5'b00000;  // 路线/业务状态“STATE / MENU”的固定编码；case 分支和状态上报必须保持一致。
          localparam [4:0] STATE_CHANGE = 5'b00001;  // 路线/业务状态“STATE / CHANGE”的固定编码；case 分支和状态上报必须保持一致。
          localparam [4:0] STATE_RUN    = 5'b00011;  // 路线/业务状态“STATE / RUN”的固定编码；case 分支和状态上报必须保持一致。
          localparam [4:0] STATE_STOP   = 5'b00100;  // 路线/业务状态“STATE / STOP”的固定编码；case 分支和状态上报必须保持一致。

          localparam [1:0] ROUTE_STOP    = 2'b00;  // 2 bit 路线命令“ROUTE / STOP”的编码，存入 route_info 对应槽位。
          localparam [1:0] ROUTE_RIGHT   = 2'b01;  // 2 bit 路线命令“ROUTE / RIGHT”的编码，存入 route_info 对应槽位。
          localparam [1:0] ROUTE_LEFT    = 2'b10;  // 2 bit 路线命令“ROUTE / LEFT”的编码，存入 route_info 对应槽位。
          localparam [1:0] ROUTE_FORWARD = 2'b11;  // 2 bit 路线命令“ROUTE / FORWARD”的编码，存入 route_info 对应槽位。

          localparam [1:0] DIR_UP    = 2'd0;  // 小车朝向“DIR / UP”的模 4 编码；加减 1 分别完成 90 度右/左转。
          localparam [1:0] DIR_RIGHT = 2'd1;  // 小车朝向“DIR / RIGHT”的模 4 编码；加减 1 分别完成 90 度右/左转。
          localparam [1:0] DIR_DOWN  = 2'd2;  // 小车朝向“DIR / DOWN”的模 4 编码；加减 1 分别完成 90 度右/左转。
          localparam [1:0] DIR_LEFT  = 2'd3;  // 小车朝向“DIR / LEFT”的模 4 编码；加减 1 分别完成 90 度右/左转。

          localparam integer ROUTE_WORDS = 8;  // 本模块实际展开/显示的 2 bit 路线命令槽数量；数组和循环边界均以此为准。

          localparam [4:0] WORD_MENU   = 5'd0;  // GUI 字符表中“WORD / MENU”文本的索引。
          localparam [4:0] WORD_EDIT   = 5'd1;  // GUI 字符表中“WORD / EDIT”文本的索引。
          localparam [4:0] WORD_RUN    = 5'd2;  // GUI 字符表中“WORD / RUN”文本的索引。
          localparam [4:0] WORD_STOP   = 5'd3;  // GUI 字符表中“WORD / STOP”文本的索引。
          localparam [4:0] WORD_CHANGE = 5'd4;  // GUI 字符表中“WORD / CHANGE”文本的索引。
          localparam [4:0] WORD_TOTAL  = 5'd5;  // GUI 字符表中“WORD / TOTAL”文本的索引。
          localparam [4:0] WORD_DONE   = 5'd6;  // GUI 字符表中“WORD / DONE”文本的索引。
          localparam [4:0] WORD_LEFT   = 5'd7;  // GUI 字符表中“WORD / LEFT”文本的索引。
          localparam [4:0] WORD_SPEED  = 5'd8;  // GUI 字符表中“WORD / SPEED”文本的索引。
          localparam [4:0] WORD_ACT    = 5'd9;  // GUI 字符表中“WORD / ACT”文本的索引。
          localparam [4:0] WORD_STEP   = 5'd10;  // GUI 字符表中“WORD / STEP”文本的索引。
          localparam [4:0] WORD_NX     = 5'd11;  // GUI 字符表中“WORD / NX”文本的索引。
          localparam [4:0] WORD_NY     = 5'd12;  // GUI 字符表中“WORD / NY”文本的索引。
          localparam [4:0] WORD_TX     = 5'd13;  // GUI 字符表中“WORD / TX”文本的索引。
          localparam [4:0] WORD_TY     = 5'd14;  // GUI 字符表中“WORD / TY”文本的索引。
          localparam [4:0] WORD_RT     = 5'd15;  // GUI 字符表中“WORD / RT”文本的索引。
          localparam [4:0] WORD_OLD    = 5'd16;  // GUI 字符表中“WORD / OLD”文本的索引。
          localparam [4:0] WORD_NEXT   = 5'd17;  // GUI 字符表中“WORD / NEXT”文本的索引。
          localparam [4:0] WORD_PATH   = 5'd18;  // GUI 字符表中“WORD / PATH”文本的索引。

          // -----------------------------------------------------------------------------
          // 内部网络、寄存器和计数器
          // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
          // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
          // -----------------------------------------------------------------------------
          integer i;  // 路线重建与逐像素命中循环下标；遍历 8 个命令槽清零线段数组、生成历史/待执行线段并测试当前像素。
          integer cur_x;  // 路线重建游标的当前屏幕 X 坐标；每生成一条前进线段后移动到线段终点。
          integer cur_y;  // 路线重建游标的当前屏幕 Y 坐标；每生成一条前进线段后移动到线段终点。
          integer next_x;  // 根据当前朝向和 step_pix 计算的下一条路线线段终点 X 坐标。
          integer next_y;  // 根据当前朝向和 step_pix 计算的下一条路线线段终点 Y 坐标。

          reg [1:0] cmd;  // 路线重建循环当前读取的 2 bit 命令临时量；分别遍历历史队列和待执行队列。
          reg [1:0] dir;  // 路线重建循环的临时朝向；遇到左/右命令旋转，遇到前进命令决定线段方向。

          reg [6:0] step_pix;  // 当前每个逻辑路线步骤对应的屏幕像素间距；随 active_route_steps 分段缩放。
          reg [8:0] total_bar_w;  // 计划总路线长度换算并限制到 0..320 后的信息条宽度，单位像素。
          reg [8:0] done_bar_w;  // 已完成路线长度换算并限制到 0..320 后的信息条宽度，单位像素。
          reg [8:0] remain_bar_w;  // 剩余路线长度换算并限制到 0..320 后的信息条宽度，单位像素。
          reg [8:0] speed_bar_w;  // 当前速度除以 2 并限制到 0..320 后的信息条宽度，单位像素。
          reg [11:0] car_x;  // 根据已完成路线重建得到的小车屏幕中心 X 坐标。
          reg [11:0] car_y;  // 根据已完成路线重建得到的小车屏幕中心 Y 坐标。
          reg [1:0] car_dir;  // 根据已完成路线重建得到的小车当前显示朝向。
          reg [11:0] target_draw_x;  // 逻辑目标 X 坐标相对 128 原点缩放后得到的屏幕标记 X 坐标。
          reg [11:0] target_draw_y;  // 逻辑目标 Y 坐标相对 128 原点缩放后得到的屏幕标记 Y 坐标。
          reg [11:0] stop_seg_sx;  // 停车警戒短线的“线段起点 X”屏幕坐标；方向由 car_dir 决定。
          reg [11:0] stop_seg_sy;  // 停车警戒短线的“线段起点 Y”屏幕坐标；方向由 car_dir 决定。
          reg [11:0] stop_seg_ex;  // 停车警戒短线的“线段终点 X”屏幕坐标；方向由 car_dir 决定。
          reg [11:0] stop_seg_ey;  // 停车警戒短线的“线段终点 Y”屏幕坐标；方向由 car_dir 决定。
          reg stop_seg_valid;  // 停车状态前方警戒短线是否存在；为 1 时 stop_seg_* 四个端点有效。

          reg [11:0] old_seg_sx [0:ROUTE_WORDS-1];  // 已完成路线每个槽位对应线段的起点 X 屏幕坐标数组。
          reg [11:0] old_seg_sy [0:ROUTE_WORDS-1];  // 已完成路线每个槽位对应线段的起点 Y 屏幕坐标数组。
          reg [11:0] old_seg_ex [0:ROUTE_WORDS-1];  // 已完成路线每个槽位对应线段的终点 X 屏幕坐标数组。
          reg [11:0] old_seg_ey [0:ROUTE_WORDS-1];  // 已完成路线每个槽位对应线段的终点 Y 屏幕坐标数组。
          reg [11:0] active_seg_sx [0:ROUTE_WORDS-1];  // 待执行路线每个槽位对应线段的起点 X 屏幕坐标数组。
          reg [11:0] active_seg_sy [0:ROUTE_WORDS-1];  // 待执行路线每个槽位对应线段的起点 Y 屏幕坐标数组。
          reg [11:0] active_seg_ex [0:ROUTE_WORDS-1];  // 待执行路线每个槽位对应线段的终点 X 屏幕坐标数组。
          reg [11:0] active_seg_ey [0:ROUTE_WORDS-1];  // 待执行路线每个槽位对应线段的终点 Y 屏幕坐标数组。
          reg [ROUTE_WORDS-1:0] old_seg_valid;  // 已完成路线各槽位是否包含一条可绘制前进线段的位图；bit i 对应第 i 个内部命令。
          reg [ROUTE_WORDS-1:0] active_seg_valid;  // 待执行路线各槽位是否包含一条可绘制前进线段的位图；bit i 对应第 i 个内部命令。

          reg old_route_pixel;  // 当前像素是否命中已完成路线中的任一虚线线段。
          reg active_route_pixel;  // 当前像素是否命中待执行路线中的任一实线线段。
          reg stop_line_pixel;  // 当前像素是否命中停车状态下绘制的前方警戒短线。
          reg [23:0] rgb_next;  // 当前像素完成背景、面板、路线、小车和文字优先级叠加后的 RGB888 组合颜色。
          reg grid_x_line;  // 当前 pixel_x 是否等于任一预设垂直网格线坐标。
          reg grid_y_line;  // 当前 pixel_y 是否等于任一预设水平网格线坐标。

          reg txt_state0;  // 当前像素是否命中“state0”对应文字字模；为 1 时文字颜色覆盖底图。
          reg txt_state1;  // 当前像素是否命中“state1”对应文字字模；为 1 时文字颜色覆盖底图。
          reg txt_state2;  // 当前像素是否命中“state2”对应文字字模；为 1 时文字颜色覆盖底图。
          reg txt_state3;  // 当前像素是否命中“state3”对应文字字模；为 1 时文字颜色覆盖底图。
          reg txt_change_box;  // 当前像素是否命中“change / BOX”对应文字字模；为 1 时文字颜色覆盖底图。
          reg txt_run_box;  // 当前像素是否命中“RUN / BOX”对应文字字模；为 1 时文字颜色覆盖底图。
          reg txt_info_dark;  // 当前像素是否命中“信息 / dark”对应文字字模；为 1 时文字颜色覆盖底图。
          reg txt_route_dark;  // 当前像素是否命中“路线 / dark”对应文字字模；为 1 时文字颜色覆盖底图。
        reg txt_route_hex;  // 当前像素是否命中“路线 / HEX”对应文字字模；为 1 时文字颜色覆盖底图。
        reg txt_menu_hint;  // 当前像素是否命中“菜单 / hint”对应文字字模；为 1 时文字颜色覆盖底图。
        reg txt_route_hint;  // 当前像素是否命中“路线 / hint”对应文字字模；为 1 时文字颜色覆盖底图。

        integer text_idx;  // 当前像素所在的 12 像素文本字符槽编号。
          integer text_base_x;  // 当前字符槽左边界的屏幕 X 坐标，传给 5x7 字模命中函数。
          reg [7:0] text_char;  // 当前文本槽查表得到的 ASCII 字符，随后交给字模函数绘制。

          wire status_panel;  // 当前 pixel_x/pixel_y 对“状态帧快照 / panel”图形区域的命中判定；为 1 时进入对应着色分支。
          wire menu_panel;  // 当前 pixel_x/pixel_y 对“菜单 / panel”图形区域的命中判定；为 1 时进入对应着色分支。
          wire route_panel;  // 当前 pixel_x/pixel_y 对“路线 / panel”图形区域的命中判定；为 1 时进入对应着色分支。
          wire change_box;  // 当前 pixel_x/pixel_y 对“change / BOX”图形区域的命中判定；为 1 时进入对应着色分支。
          wire run_box;  // 当前 pixel_x/pixel_y 对“RUN / BOX”图形区域的命中判定；为 1 时进入对应着色分支。
          wire map_area;  // 当前 pixel_x/pixel_y 对“MAP / area”图形区域的命中判定；为 1 时进入对应着色分支。
          wire border_line;  // 当前 pixel_x/pixel_y 对“border / 输出线”图形区域的命中判定；为 1 时进入对应着色分支。
          wire grid_line;  // 当前 pixel_x/pixel_y 对“grid / 输出线”图形区域的命中判定；为 1 时进入对应着色分支。
          wire change_select;  // menu_info 当前是否选中/进入路线编辑项；决定 EDIT 菜单框高亮。
          wire run_select;  // menu_info 当前是否选中/进入运行项；决定 RUN 菜单框高亮。
          wire state_stop;  // state_info 是否等于 STATE_STOP 的组合判定；控制停车警戒线和 STOP 状态配色。
          wire state_menu_sel;  // state_info 是否等于 STATE_MENU 的组合判定；选择菜单状态栏和文字。
          wire state_change_sel;  // state_info 是否等于 STATE_CHANGE 的组合判定；选择路线编辑状态栏和文字。
          wire state_run_sel;  // state_info 是否等于 STATE_RUN 的组合判定；选择运行状态栏和文字。
          wire state_stop_sel;  // state_info 是否等于 STATE_STOP 的组合判定；选择停车状态栏和文字。
          wire state_bar_0;  // 当前 pixel_x/pixel_y 对“状态 / 信息条 / 0”图形区域的命中判定；为 1 时进入对应着色分支。
          wire state_bar_1;  // 当前 pixel_x/pixel_y 对“状态 / 信息条 / 1”图形区域的命中判定；为 1 时进入对应着色分支。
          wire state_bar_2;  // 当前 pixel_x/pixel_y 对“状态 / 信息条 / 2”图形区域的命中判定；为 1 时进入对应着色分支。
          wire state_bar_3;  // 当前 pixel_x/pixel_y 对“状态 / 信息条 / 3”图形区域的命中判定；为 1 时进入对应着色分支。
          wire bt_status_box;  // 当前 pixel_x/pixel_y 对“BT / 状态帧快照 / BOX”图形区域的命中判定；为 1 时进入对应着色分支。
          wire info_bar_total;  // 当前 pixel_x/pixel_y 对“信息 / 信息条 / 总计”图形区域的命中判定；为 1 时进入对应着色分支。
          wire info_bar_done;  // 当前 pixel_x/pixel_y 对“信息 / 信息条 / 完成标志”图形区域的命中判定；为 1 时进入对应着色分支。
          wire info_bar_remain;  // 当前 pixel_x/pixel_y 对“信息 / 信息条 / 剩余量”图形区域的命中判定；为 1 时进入对应着色分支。
          wire info_bar_speed;  // 当前 pixel_x/pixel_y 对“信息 / 信息条 / 速度”图形区域的命中判定；为 1 时进入对应着色分支。
          wire car_body;  // 当前 pixel_x/pixel_y 对“小车 / 车体”图形区域的命中判定；为 1 时进入对应着色分支。
          wire car_arrow;  // 当前 pixel_x/pixel_y 对“小车 / 方向箭头”图形区域的命中判定；为 1 时进入对应着色分支。
          wire target_marker;  // 当前 pixel_x/pixel_y 对“目标 / 标记”图形区域的命中判定；为 1 时进入对应着色分支。
          wire now_marker;  // 当前 pixel_x/pixel_y 对“当前位置 / 标记”图形区域的命中判定；为 1 时进入对应着色分支。

          // -----------------------------------------------------------------------------
          // [FUNCTION] 辅助函数: calc_step_pix
          // 根据路线步数分段选择 58/44/34/26/20/16 像素网格间距，使完整路线尽量落入屏幕地图区。
          // -----------------------------------------------------------------------------
          function integer calc_step_pix;
            input [5:0] steps;  // 6 bit 函数 `calc_step_pix` 的“步数”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            begin
              if (steps <= 4)
                calc_step_pix = 58;
              else if (steps <= 8)
                calc_step_pix = 44;
              else if (steps <= 12)
                calc_step_pix = 34;
              else if (steps <= 18)
                calc_step_pix = 26;
              else if (steps <= 24)
                calc_step_pix = 20;
              else
                calc_step_pix = 16;
            end
          endfunction

          // -----------------------------------------------------------------------------
          // [FUNCTION] 辅助函数: clamp_bar_w
          // 把信息条宽度限制在 0..320 像素，防止数值超出右侧信息栏边界。
          // -----------------------------------------------------------------------------
          function integer clamp_bar_w;
            input [15:0] value;  // 16 bit 函数 `clamp_bar_w` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            integer bar_w;  // 把输入路线/速度量复制成整数后限制到 0..320，所得值直接作为信息条像素宽度。 该变量只在本次函数求值期间存在，不保存跨周期状态。
            begin
              bar_w = value;
              if (bar_w > 320)
                bar_w = 320;
              if (bar_w < 0)
                bar_w = 0;
              clamp_bar_w = bar_w;
            end
          endfunction

          // -----------------------------------------------------------------------------
          // [FUNCTION] 辅助函数: scaled_offset
          // 把以 128 为原点的逻辑坐标差按当前网格间距换算为有符号像素偏移；乘法用移位相加实现。
          // -----------------------------------------------------------------------------
          function integer scaled_offset;
            input [7:0] value;  // 8 bit 函数 `scaled_offset` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input integer pix;  // 函数 `scaled_offset` 的“PIX”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            integer diff;  // 相对逻辑原点 128 的最终有符号像素偏移；value<128 时取负，反之取正。 该变量只在本次函数求值期间存在，不保存跨周期状态。
            integer mag;  // 逻辑坐标 value 与原点 128 之间的无符号格数差。 该变量只在本次函数求值期间存在，不保存跨周期状态。
            integer scaled;  // mag 按当前 pix 网格间距用移位加法放大后的无符号像素距离。 该变量只在本次函数求值期间存在，不保存跨周期状态。
            begin
              if (value >= 8'd128)
                mag = value - 8'd128;
              else
                mag = 8'd128 - value;

              case (pix)
                58: scaled = (mag << 5) + (mag << 4) + (mag << 3) + (mag << 1);
                44: scaled = (mag << 5) + (mag << 3) + (mag << 2);
                34: scaled = (mag << 5) + (mag << 1);
                26: scaled = (mag << 4) + (mag << 3) + (mag << 1);
                20: scaled = (mag << 4) + (mag << 2);
                default: scaled = mag << 4;
              endcase

              if (value >= 8'd128)
                diff = scaled;
              else
                diff = 0 - scaled;

              scaled_offset = diff;
            end
          endfunction

          // -----------------------------------------------------------------------------
          // [FUNCTION] 辅助函数: segment_hit
          // 判断当前 pixel_x/pixel_y 是否落在给定水平或垂直路线线段及其线宽范围内。
          // -----------------------------------------------------------------------------
          function segment_hit;
            input integer sx;  // 函数 `segment_hit` 的“线段起点 X”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input integer sy;  // 函数 `segment_hit` 的“线段起点 Y”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input integer ex;  // 函数 `segment_hit` 的“线段终点 X”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input integer ey;  // 函数 `segment_hit` 的“线段终点 Y”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input integer width;  // 函数 `segment_hit` 的“线宽”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            integer lx;  // 线段两个 X 端点中的较小值，用于建立带 width 余量的水平包围盒。 该变量只在本次函数求值期间存在，不保存跨周期状态。
            integer hx;  // 线段两个 X 端点中的较大值，用于建立带 width 余量的水平包围盒。 该变量只在本次函数求值期间存在，不保存跨周期状态。
            integer ly;  // 线段两个 Y 端点中的较小值，用于建立带 width 余量的垂直包围盒。 该变量只在本次函数求值期间存在，不保存跨周期状态。
            integer hy;  // 线段两个 Y 端点中的较大值，用于建立带 width 余量的垂直包围盒。 该变量只在本次函数求值期间存在，不保存跨周期状态。
            begin
              if (sx < ex) begin lx = sx; hx = ex; end else begin lx = ex; hx = sx; end
              if (sy < ey) begin ly = sy; hy = ey; end else begin ly = ey; hy = sy; end
              segment_hit = (pixel_x >= lx - width) && (pixel_x <= hx + width) &&
                            (pixel_y >= ly - width) && (pixel_y <= hy + width) &&
                            (((sx == ex) && (pixel_x >= sx - width) && (pixel_x <= sx + width)) ||
                             ((sy == ey) && (pixel_y >= sy - width) && (pixel_y <= sy + width)));
            end
          endfunction

          // -----------------------------------------------------------------------------
          // [FUNCTION] 辅助函数: glyph5x7_2x_hit
          // 查询内置 5x7 字模并放大到 10x14 像素，返回当前像素是否应点亮。
          // -----------------------------------------------------------------------------
          function glyph5x7_2x_hit;
            input [7:0] ch;  // 8 bit 函数 `glyph5x7_2x_hit` 的“待显示字符”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input [11:0] px;  // 12 bit 函数 `glyph5x7_2x_hit` 的“当前像素 X”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input [11:0] py;  // 12 bit 函数 `glyph5x7_2x_hit` 的“当前像素 Y”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input integer base_x;  // 函数 `glyph5x7_2x_hit` 的“基准坐标 / X 坐标”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input integer base_y;  // 函数 `glyph5x7_2x_hit` 的“基准坐标 / Y 坐标”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            reg [2:0] row;  // 3 bit 当前 py 在 2 倍放大 5x7 字模中的行号 0..6；每两个屏幕像素对应一行。 该变量只在本次函数求值期间存在，不保存跨周期状态。
            reg [2:0] col;  // 3 bit 当前 px 在 2 倍放大字模中的反向列号 4..0，用于索引 bits。 该变量只在本次函数求值期间存在，不保存跨周期状态。
            reg [4:0] bits;  // 5 bit 按字符 ch 和 row 查出的当前 5 像素字模行；bits[col] 决定当前像素是否点亮。 该变量只在本次函数求值期间存在，不保存跨周期状态。
            begin
              glyph5x7_2x_hit = 1'b0;
              bits = 5'b00000;
              if ((ch != " ") &&
                  (px >= base_x) && (px < base_x + 10) &&
                  (py >= base_y) && (py < base_y + 14)) begin
                row = (py - base_y) >> 1;
                case (ch)
                  "0": case (row) 0: bits=5'b01110; 1: bits=5'b10011; 2: bits=5'b10101; 3: bits=5'b11001; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b01110; endcase
                  "1": case (row) 0: bits=5'b00100; 1: bits=5'b01100; 2: bits=5'b00100; 3: bits=5'b00100; 4: bits=5'b00100; 5: bits=5'b00100; default: bits=5'b01110; endcase
                  "2": case (row) 0: bits=5'b01110; 1: bits=5'b10001; 2: bits=5'b00001; 3: bits=5'b00010; 4: bits=5'b00100; 5: bits=5'b01000; default: bits=5'b11111; endcase
                  "3": case (row) 0: bits=5'b11110; 1: bits=5'b00001; 2: bits=5'b00001; 3: bits=5'b01110; 4: bits=5'b00001; 5: bits=5'b00001; default: bits=5'b11110; endcase
                  "4": case (row) 0: bits=5'b00010; 1: bits=5'b00110; 2: bits=5'b01010; 3: bits=5'b10010; 4: bits=5'b11111; 5: bits=5'b00010; default: bits=5'b00010; endcase
                  "5": case (row) 0: bits=5'b11111; 1: bits=5'b10000; 2: bits=5'b10000; 3: bits=5'b11110; 4: bits=5'b00001; 5: bits=5'b00001; default: bits=5'b11110; endcase
                  "6": case (row) 0: bits=5'b01110; 1: bits=5'b10000; 2: bits=5'b10000; 3: bits=5'b11110; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b01110; endcase
                  "7": case (row) 0: bits=5'b11111; 1: bits=5'b00001; 2: bits=5'b00010; 3: bits=5'b00100; 4: bits=5'b01000; 5: bits=5'b01000; default: bits=5'b01000; endcase
                  "8": case (row) 0: bits=5'b01110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b01110; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b01110; endcase
                "9": case (row) 0: bits=5'b01110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b01111; 4: bits=5'b00001; 5: bits=5'b00001; default: bits=5'b01110; endcase
                "A": case (row) 0: bits=5'b01110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b11111; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b10001; endcase
                "C": case (row) 0: bits=5'b01111; 1: bits=5'b10000; 2: bits=5'b10000; 3: bits=5'b10000; 4: bits=5'b10000; 5: bits=5'b10000; default: bits=5'b01111; endcase
                  "D": case (row) 0: bits=5'b11110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b10001; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b11110; endcase
                  "E": case (row) 0: bits=5'b11111; 1: bits=5'b10000; 2: bits=5'b11110; 3: bits=5'b10000; 4: bits=5'b10000; 5: bits=5'b10000; default: bits=5'b11111; endcase
                  "F": case (row) 0: bits=5'b11111; 1: bits=5'b10000; 2: bits=5'b11110; 3: bits=5'b10000; 4: bits=5'b10000; 5: bits=5'b10000; default: bits=5'b10000; endcase
                  "G": case (row) 0: bits=5'b01111; 1: bits=5'b10000; 2: bits=5'b10000; 3: bits=5'b10111; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b01111; endcase
                  "H": case (row) 0: bits=5'b10001; 1: bits=5'b10001; 2: bits=5'b11111; 3: bits=5'b10001; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b10001; endcase
                  "I": case (row) 0: bits=5'b11111; 1: bits=5'b00100; 2: bits=5'b00100; 3: bits=5'b00100; 4: bits=5'b00100; 5: bits=5'b00100; default: bits=5'b11111; endcase
                  "L": case (row) 0: bits=5'b10000; 1: bits=5'b10000; 2: bits=5'b10000; 3: bits=5'b10000; 4: bits=5'b10000; 5: bits=5'b10000; default: bits=5'b11111; endcase
                  "M": case (row) 0: bits=5'b10001; 1: bits=5'b11011; 2: bits=5'b10101; 3: bits=5'b10101; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b10001; endcase
                  "N": case (row) 0: bits=5'b10001; 1: bits=5'b11001; 2: bits=5'b10101; 3: bits=5'b10011; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b10001; endcase
                  "O": case (row) 0: bits=5'b01110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b10001; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b01110; endcase
                  "P": case (row) 0: bits=5'b11110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b11110; 4: bits=5'b10000; 5: bits=5'b10000; default: bits=5'b10000; endcase
                  "R": case (row) 0: bits=5'b11110; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b11110; 4: bits=5'b10100; 5: bits=5'b10010; default: bits=5'b10001; endcase
                  "S": case (row) 0: bits=5'b01111; 1: bits=5'b10000; 2: bits=5'b10000; 3: bits=5'b01110; 4: bits=5'b00001; 5: bits=5'b00001; default: bits=5'b11110; endcase
                  "T": case (row) 0: bits=5'b11111; 1: bits=5'b00100; 2: bits=5'b00100; 3: bits=5'b00100; 4: bits=5'b00100; 5: bits=5'b00100; default: bits=5'b00100; endcase
                  "U": case (row) 0: bits=5'b10001; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b10001; 4: bits=5'b10001; 5: bits=5'b10001; default: bits=5'b01110; endcase
                  "W": case (row) 0: bits=5'b10001; 1: bits=5'b10001; 2: bits=5'b10001; 3: bits=5'b10101; 4: bits=5'b10101; 5: bits=5'b10101; default: bits=5'b01010; endcase
                  "X": case (row) 0: bits=5'b10001; 1: bits=5'b01010; 2: bits=5'b00100; 3: bits=5'b00100; 4: bits=5'b00100; 5: bits=5'b01010; default: bits=5'b10001; endcase
                  "Y": case (row) 0: bits=5'b10001; 1: bits=5'b01010; 2: bits=5'b00100; 3: bits=5'b00100; 4: bits=5'b00100; 5: bits=5'b00100; default: bits=5'b00100; endcase
                  default: bits = 5'b00000;
                endcase
                col = 4 - ((px - base_x) >> 1);
                glyph5x7_2x_hit = bits[col];
              end
            end
          endfunction

          // -----------------------------------------------------------------------------
          // [FUNCTION] 辅助函数: dec5_char
          // 用重复减法把 16 bit 数值拆成五位十进制，并按 idx 返回指定 ASCII 数字。
          // -----------------------------------------------------------------------------
          function [7:0] dec5_char;
            input [15:0] value;  // 16 bit 函数 `dec5_char` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input [2:0] idx;  // 3 bit 函数 `dec5_char` 的“字符位置”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            integer digit;  // 函数 `dec5_char` 根据 idx 选中的最终十进制数字 0..9；末尾加 '0' 转成 ASCII。
            integer rem;  // 函数 `dec5_char` 逐级减去高位十进制权值后保留的余数；最终用于计算较低位。
            integer d0;  // 函数 `dec5_char` 拆分得到的万位数字 0..9；由重复减法累计得到。
            integer d1;  // 函数 `dec5_char` 拆分得到的千位数字 0..9；由重复减法累计得到。
            integer d2;  // 函数 `dec5_char` 拆分得到的百位数字 0..9；由重复减法累计得到。
            integer d3;  // 函数 `dec5_char` 拆分得到的十位数字 0..9；由重复减法累计得到。
            integer d4;  // 函数 `dec5_char` 拆分得到的个位数字 0..9；由重复减法累计得到。
            integer n;  // 函数 `dec5_char` 重复减法循环的展开下标；只控制本次十进制拆位次数，不是运行时状态计数器。
            begin
              rem = value;
              d0 = 0;
              d1 = 0;
              d2 = 0;
              d3 = 0;
              d4 = 0;
              for (n = 0; n < 6; n = n + 1)
                if (rem >= 10000) begin rem = rem - 10000; d0 = d0 + 1; end
              for (n = 0; n < 10; n = n + 1)
                if (rem >= 1000) begin rem = rem - 1000; d1 = d1 + 1; end
              for (n = 0; n < 10; n = n + 1)
                if (rem >= 100) begin rem = rem - 100; d2 = d2 + 1; end
              for (n = 0; n < 10; n = n + 1)
                if (rem >= 10) begin rem = rem - 10; d3 = d3 + 1; end
              d4 = rem;
              case (idx)
                3'd0: digit = d0;
                3'd1: digit = d1;
                3'd2: digit = d2;
                3'd3: digit = d3;
                default: digit = d4;
              endcase
              dec5_char = 8'd48 + digit;
            end
          endfunction

          // -----------------------------------------------------------------------------
          // [FUNCTION] 辅助函数: dec3_char
          // 把 8 bit 数值拆成三位十进制，并按 idx 返回百/十/个位 ASCII 数字。
          // -----------------------------------------------------------------------------
          function [7:0] dec3_char;
            input [7:0] value;  // 8 bit 函数 `dec3_char` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input [1:0] idx;  // 2 bit 函数 `dec3_char` 的“字符位置”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            integer digit;  // 函数 `dec3_char` 根据 idx 选中的最终十进制数字 0..9；末尾加 '0' 转成 ASCII。
            integer rem;  // 函数 `dec3_char` 逐级减去高位十进制权值后保留的余数；最终用于计算较低位。
            integer d0;  // 函数 `dec3_char` 拆分得到的百位数字 0..9；由重复减法累计得到。
            integer d1;  // 函数 `dec3_char` 拆分得到的十位数字 0..9；由重复减法累计得到。
            integer d2;  // 函数 `dec3_char` 拆分得到的个位数字 0..9；由重复减法累计得到。
            integer n;  // 函数 `dec3_char` 重复减法循环的展开下标；只控制本次十进制拆位次数，不是运行时状态计数器。
            begin
              rem = value;
              d0 = 0;
              d1 = 0;
              d2 = 0;
              for (n = 0; n < 2; n = n + 1)
                if (rem >= 100) begin rem = rem - 100; d0 = d0 + 1; end
              for (n = 0; n < 10; n = n + 1)
                if (rem >= 10) begin rem = rem - 10; d1 = d1 + 1; end
              d2 = rem;
              case (idx)
                2'd0: digit = d0;
                2'd1: digit = d1;
                default: digit = d2;
              endcase
              dec3_char = 8'd48 + digit;
            end
          endfunction

          // -----------------------------------------------------------------------------
          // [FUNCTION] 辅助函数: dec2_char
          // 把 8 bit 数值拆成两位十进制，并按 idx 返回十位或个位 ASCII 数字。
          // -----------------------------------------------------------------------------
          function [7:0] dec2_char;
            input [7:0] value;  // 8 bit 函数 `dec2_char` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input integer idx;  // 函数 `dec2_char` 的“字符位置”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            integer digit;  // 函数 `dec2_char` 根据 idx 选中的最终十进制数字 0..9；末尾加 '0' 转成 ASCII。
            integer rem;  // 函数 `dec2_char` 逐级减去高位十进制权值后保留的余数；最终用于计算较低位。
            integer d0;  // 函数 `dec2_char` 拆分得到的十位数字 0..9；由重复减法累计得到。
            integer d1;  // 函数 `dec2_char` 拆分得到的个位数字 0..9；由重复减法累计得到。
            integer n;  // 函数 `dec2_char` 重复减法循环的展开下标；只控制本次十进制拆位次数，不是运行时状态计数器。
            begin
              rem = value;
              d0 = 0;
              d1 = 0;
              for (n = 0; n < 10; n = n + 1)
                if (rem >= 10) begin rem = rem - 10; d0 = d0 + 1; end
              d1 = rem;
              if (idx == 0)
                digit = d0;
              else
                digit = d1;
              dec2_char = 8'd48 + digit;
            end
          endfunction

          // -----------------------------------------------------------------------------
          // [FUNCTION] 辅助函数: slot12
          // 把水平像素偏移按每 12 像素一格量化到 0..15 字符槽位。
          // -----------------------------------------------------------------------------
          function integer slot12;
            input integer offset;  // 函数 `slot12` 的“偏移量”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            begin
              if (offset < 12)
                slot12 = 0;
              else if (offset < 24)
                slot12 = 1;
              else if (offset < 36)
                slot12 = 2;
              else if (offset < 48)
                slot12 = 3;
              else if (offset < 60)
                slot12 = 4;
              else if (offset < 72)
                slot12 = 5;
              else if (offset < 84)
                slot12 = 6;
              else if (offset < 96)
                slot12 = 7;
              else if (offset < 108)
                slot12 = 8;
              else if (offset < 120)
                slot12 = 9;
              else if (offset < 132)
                slot12 = 10;
              else if (offset < 144)
                slot12 = 11;
              else if (offset < 156)
                slot12 = 12;
              else if (offset < 168)
                slot12 = 13;
              else if (offset < 180)
                slot12 = 14;
              else
                slot12 = 15;
            end
          endfunction

          // -----------------------------------------------------------------------------
          // [FUNCTION] 辅助函数: hex16_char
          // 从 64 bit 路线字段中按高位优先选择一个半字节并输出大写十六进制字符。
          // -----------------------------------------------------------------------------
          function [7:0] hex16_char;
            input [63:0] value;  // 64 bit 函数 `hex16_char` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input integer idx;  // 函数 `hex16_char` 的“字符位置”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            reg [3:0] nibble;  // 4 bit 从 64 bit value 中按 idx 选出的 4 bit 半字节；小于 10 映射 '0'..'9'，否则映射 'A'..'F'。 该变量只在本次函数求值期间存在，不保存跨周期状态。
            begin
              nibble = value[((15 - idx) * 4) +: 4];
              if (nibble < 10)
                hex16_char = 8'd48 + nibble;
              else
                hex16_char = 8'd55 + nibble;
            end
          endfunction

          // -----------------------------------------------------------------------------
          // [FUNCTION] 辅助函数: text_word_char
          // 根据 WORD_* 文本编号和字符下标查表输出菜单/状态英文字符。
          // -----------------------------------------------------------------------------
          function [7:0] text_word_char;
            input [4:0] word;  // 5 bit 函数 `text_word_char` 的“字表项”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input integer idx;  // 函数 `text_word_char` 的“字符位置”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            begin
              text_word_char = " ";
              case (word)
                WORD_MENU:   case (idx) 0: text_word_char="M"; 1: text_word_char="E"; 2: text_word_char="N"; 3: text_word_char="U"; endcase
                WORD_EDIT:   case (idx) 0: text_word_char="E"; 1: text_word_char="D"; 2: text_word_char="I"; 3: text_word_char="T"; endcase
                WORD_RUN:    case (idx) 0: text_word_char="R"; 1: text_word_char="U"; 2: text_word_char="N"; endcase
                WORD_STOP:   case (idx) 0: text_word_char="S"; 1: text_word_char="T"; 2: text_word_char="O"; 3: text_word_char="P"; endcase
                WORD_CHANGE: case (idx) 0: text_word_char="C"; 1: text_word_char="H"; 2: text_word_char="A"; 3: text_word_char="N"; 4: text_word_char="G"; 5: text_word_char="E"; endcase
                WORD_TOTAL:  case (idx) 0: text_word_char="T"; 1: text_word_char="O"; 2: text_word_char="T"; 3: text_word_char="A"; 4: text_word_char="L"; endcase
                WORD_DONE:   case (idx) 0: text_word_char="D"; 1: text_word_char="O"; 2: text_word_char="N"; 3: text_word_char="E"; endcase
                WORD_LEFT:   case (idx) 0: text_word_char="L"; 1: text_word_char="E"; 2: text_word_char="F"; 3: text_word_char="T"; endcase
                WORD_SPEED:  case (idx) 0: text_word_char="S"; 1: text_word_char="P"; 2: text_word_char="E"; 3: text_word_char="E"; 4: text_word_char="D"; endcase
                WORD_ACT:    case (idx) 0: text_word_char="A"; 1: text_word_char="C"; 2: text_word_char="T"; endcase
                WORD_STEP:   case (idx) 0: text_word_char="S"; 1: text_word_char="T"; 2: text_word_char="E"; 3: text_word_char="P"; endcase
                WORD_NX:     case (idx) 0: text_word_char="N"; 1: text_word_char="X"; endcase
                WORD_NY:     case (idx) 0: text_word_char="N"; 1: text_word_char="Y"; endcase
                WORD_TX:     case (idx) 0: text_word_char="T"; 1: text_word_char="X"; endcase
                WORD_TY:     case (idx) 0: text_word_char="T"; 1: text_word_char="Y"; endcase
                WORD_RT:     case (idx) 0: text_word_char="R"; 1: text_word_char="T"; endcase
                WORD_OLD:    case (idx) 0: text_word_char="O"; 1: text_word_char="L"; 2: text_word_char="D"; endcase
                WORD_NEXT:   case (idx) 0: text_word_char="N"; 1: text_word_char="E"; 2: text_word_char="X"; 3: text_word_char="T"; endcase
                WORD_PATH:   case (idx) 0: text_word_char="P"; 1: text_word_char="A"; 2: text_word_char="T"; 3: text_word_char="H"; endcase
              endcase
            end
          endfunction

          // -----------------------------------------------------------------------------
          // [FUNCTION] 辅助函数: cmd_word_char
          // 把 2 bit 路线命令转换为 LEFT/RIGHT/FWD/STOP 四字符显示文本。
          // -----------------------------------------------------------------------------
          function [7:0] cmd_word_char;
            input [1:0] route_cmd;  // 2 bit 函数 `cmd_word_char` 的“路线 / 命令”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            input integer idx;  // 函数 `cmd_word_char` 的“字符位置”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
            begin
              cmd_word_char = " ";
              case (route_cmd)
                ROUTE_LEFT:
                  case (idx) 0: cmd_word_char="L"; 1: cmd_word_char="E"; 2: cmd_word_char="F"; 3: cmd_word_char="T"; endcase
                ROUTE_RIGHT:
                  case (idx) 0: cmd_word_char="R"; 1: cmd_word_char="I"; 2: cmd_word_char="G"; 3: cmd_word_char="H"; endcase
                ROUTE_FORWARD:
                  case (idx) 0: cmd_word_char="F"; 1: cmd_word_char="W"; 2: cmd_word_char="D"; 3: cmd_word_char=" "; endcase
                default:
                  case (idx) 0: cmd_word_char="S"; 1: cmd_word_char="T"; 2: cmd_word_char="O"; 3: cmd_word_char="P"; endcase
              endcase
            end
          endfunction

          // -----------------------------------------------------------------------------
          // 组合派生信号
          // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
          // -----------------------------------------------------------------------------
          // [ASSIGN] 组合定义“state_stop”：state_info 是否等于 STATE_STOP 的组合判定；控制停车警戒线和 STOP 状态配色。 该组合结果直接由 `state_info`（状态 / 信息）、`STATE_STOP`（STATE / STOP）的当前值决定，不引入时钟周期延迟。
          assign state_stop = (state_info == STATE_STOP);
          // [ASSIGN] 组合定义“state_menu_sel”：state_info 是否等于 STATE_MENU 的组合判定；选择菜单状态栏和文字。 该组合结果直接由 `state_info`（状态 / 信息）、`STATE_MENU`（STATE / MENU）的当前值决定，不引入时钟周期延迟。
          assign state_menu_sel   = (state_info == STATE_MENU);
          // [ASSIGN] 组合定义“state_change_sel”：state_info 是否等于 STATE_CHANGE 的组合判定；选择路线编辑状态栏和文字。 该组合结果直接由 `state_info`（状态 / 信息）、`STATE_CHANGE`（STATE / CHANGE）的当前值决定，不引入时钟周期延迟。
          assign state_change_sel = (state_info == STATE_CHANGE);
          // [ASSIGN] 组合定义“state_run_sel”：state_info 是否等于 STATE_RUN 的组合判定；选择运行状态栏和文字。 该组合结果直接由 `state_info`（状态 / 信息）、`STATE_RUN`（STATE / RUN）的当前值决定，不引入时钟周期延迟。
          assign state_run_sel    = (state_info == STATE_RUN);
          // [ASSIGN] 组合定义“state_stop_sel”：state_info 是否等于 STATE_STOP 的组合判定；选择停车状态栏和文字。 该组合结果直接由 `state_info`（状态 / 信息）、`STATE_STOP`（STATE / STOP）的当前值决定，不引入时钟周期延迟。
          assign state_stop_sel   = (state_info == STATE_STOP);

          // [ASSIGN] 组合定义“status_panel”：当前 pixel_x/pixel_y 对“状态帧快照 / panel”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `pixel_x`（像素 / X 坐标）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign status_panel = (pixel_x >= 12'd20) && (pixel_x < 12'd1000) &&
                                (pixel_y >= 12'd16) && (pixel_y < 12'd128);
          // [ASSIGN] 组合定义“menu_panel”：当前 pixel_x/pixel_y 对“菜单 / panel”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `pixel_x`（像素 / X 坐标）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign menu_panel  = (pixel_x >= 12'd40)  && (pixel_x < 12'd330) &&
                               (pixel_y >= 12'd150) && (pixel_y < 12'd590);
          // [ASSIGN] 组合定义“route_panel”：当前 pixel_x/pixel_y 对“路线 / panel”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `pixel_x`（像素 / X 坐标）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign route_panel = (pixel_x >= 12'd360) && (pixel_x < 12'd990) &&
                               (pixel_y >= 12'd150) && (pixel_y < 12'd590);
          // [ASSIGN] 组合定义“change_box”：当前 pixel_x/pixel_y 对“change / BOX”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `pixel_x`（像素 / X 坐标）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign change_box = (pixel_x >= 12'd78) && (pixel_x < 12'd292) &&
                              (pixel_y >= 12'd230) && (pixel_y < 12'd305);
          // [ASSIGN] 组合定义“run_box”：当前 pixel_x/pixel_y 对“RUN / BOX”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `pixel_x`（像素 / X 坐标）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign run_box    = (pixel_x >= 12'd78) && (pixel_x < 12'd292) &&
                              (pixel_y >= 12'd360) && (pixel_y < 12'd435);
          // [ASSIGN] 组合定义“map_area”：当前 pixel_x/pixel_y 对“MAP / area”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `pixel_x`（像素 / X 坐标）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign map_area = (pixel_x >= 12'd405) && (pixel_x < 12'd945) &&
                            (pixel_y >= 12'd180) && (pixel_y < 12'd560);

          // [ASSIGN] 组合定义“change_select”：menu_info 当前是否选中/进入路线编辑项；决定 EDIT 菜单框高亮。 该组合结果直接由 `menu_info`（菜单 / 信息）、`MENU_FOCUS_CHANGE`（MENU / FOCUS / CHANGE）、`MENU_IN_CHANGE`（MENU / IN / CHANGE）的当前值决定，不引入时钟周期延迟。
          assign change_select = (menu_info == MENU_FOCUS_CHANGE) || (menu_info == MENU_IN_CHANGE);
          // [ASSIGN] 组合定义“run_select”：menu_info 当前是否选中/进入运行项；决定 RUN 菜单框高亮。 该组合结果直接由 `menu_info`（菜单 / 信息）、`MENU_FOCUS_RUN`（MENU / FOCUS / RUN）、`MENU_IN_RUN`（MENU / IN / RUN）的当前值决定，不引入时钟周期延迟。
          assign run_select = (menu_info == MENU_FOCUS_RUN) || (menu_info == MENU_IN_RUN);

          // [ASSIGN] 组合定义“border_line”：当前 pixel_x/pixel_y 对“border / 输出线”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `pixel_x`（像素 / X 坐标）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign border_line =
              ((pixel_x == 12'd20 || pixel_x == 12'd999) && (pixel_y >= 12'd16 && pixel_y < 12'd128)) ||
              ((pixel_y == 12'd16 || pixel_y == 12'd127) && (pixel_x >= 12'd20 && pixel_x < 12'd1000)) ||
              ((pixel_x == 12'd40 || pixel_x == 12'd329) && (pixel_y >= 12'd150 && pixel_y < 12'd590)) ||
              ((pixel_y == 12'd150 || pixel_y == 12'd589) && (pixel_x >= 12'd40 && pixel_x < 12'd330)) ||
              ((pixel_x == 12'd360 || pixel_x == 12'd989) && (pixel_y >= 12'd150 && pixel_y < 12'd590)) ||
              ((pixel_y == 12'd150 || pixel_y == 12'd589) && (pixel_x >= 12'd360 && pixel_x < 12'd990)) ||
              ((pixel_x == 12'd78 || pixel_x == 12'd291) && (pixel_y >= 12'd230 && pixel_y < 12'd305)) ||
              ((pixel_y == 12'd230 || pixel_y == 12'd304) && (pixel_x >= 12'd78 && pixel_x < 12'd292)) ||
              ((pixel_x == 12'd78 || pixel_x == 12'd291) && (pixel_y >= 12'd360 && pixel_y < 12'd435)) ||
              ((pixel_y == 12'd360 || pixel_y == 12'd434) && (pixel_x >= 12'd78 && pixel_x < 12'd292)) ||
              ((pixel_x == 12'd405 || pixel_x == 12'd944) && (pixel_y >= 12'd180 && pixel_y < 12'd560)) ||
              ((pixel_y == 12'd180 || pixel_y == 12'd559) && (pixel_x >= 12'd405 && pixel_x < 12'd945));

          // [ASSIGN] 组合定义“grid_line”：当前 pixel_x/pixel_y 对“grid / 输出线”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `map_area`（MAP / area）、`grid_x_line`（grid / X 坐标 / 输出线）、`grid_y_line`（grid / Y 坐标 / 输出线）的当前值决定，不引入时钟周期延迟。
          assign grid_line = map_area && (grid_x_line || grid_y_line);

          // [ASSIGN] 组合定义“state_bar_0”：当前 pixel_x/pixel_y 对“状态 / 信息条 / 0”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `status_panel`（状态帧快照 / panel）、`pixel_x`（像素 / X 坐标）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign state_bar_0 = status_panel &&
                               (pixel_x >= 12'd52) && (pixel_x < 12'd172) &&
                               (pixel_y >= 12'd34) && (pixel_y < 12'd56);
          // [ASSIGN] 组合定义“state_bar_1”：当前 pixel_x/pixel_y 对“状态 / 信息条 / 1”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `status_panel`（状态帧快照 / panel）、`pixel_x`（像素 / X 坐标）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign state_bar_1 = status_panel &&
                               (pixel_x >= 12'd192) && (pixel_x < 12'd312) &&
                               (pixel_y >= 12'd34) && (pixel_y < 12'd56);
          // [ASSIGN] 组合定义“state_bar_2”：当前 pixel_x/pixel_y 对“状态 / 信息条 / 2”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `status_panel`（状态帧快照 / panel）、`pixel_x`（像素 / X 坐标）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign state_bar_2 = status_panel &&
                               (pixel_x >= 12'd332) && (pixel_x < 12'd452) &&
                               (pixel_y >= 12'd34) && (pixel_y < 12'd56);
          // [ASSIGN] 组合定义“state_bar_3”：当前 pixel_x/pixel_y 对“状态 / 信息条 / 3”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `status_panel`（状态帧快照 / panel）、`pixel_x`（像素 / X 坐标）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign state_bar_3 = status_panel &&
                               (pixel_x >= 12'd472) && (pixel_x < 12'd592) &&
                               (pixel_y >= 12'd34) && (pixel_y < 12'd56);
          // [ASSIGN] 组合定义“bt_status_box”：当前 pixel_x/pixel_y 对“BT / 状态帧快照 / BOX”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `status_panel`（状态帧快照 / panel）、`pixel_x`（像素 / X 坐标）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign bt_status_box = status_panel &&
                                 (pixel_x >= 12'd52) && (pixel_x < 12'd136) &&
                                 (pixel_y >= 12'd78) && (pixel_y < 12'd102);

          // [ASSIGN] 组合定义“info_bar_total”：当前 pixel_x/pixel_y 对“信息 / 信息条 / 总计”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `status_panel`（状态帧快照 / panel）、`pixel_x`（像素 / X 坐标）、`total_bar_w`（总计 / 信息条 / W）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign info_bar_total = status_panel &&
                                  (pixel_x >= 12'd640) && (pixel_x < 12'd640 + total_bar_w) &&
                                  (pixel_y >= 12'd30) && (pixel_y < 12'd42);
          // [ASSIGN] 组合定义“info_bar_done”：当前 pixel_x/pixel_y 对“信息 / 信息条 / 完成标志”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `status_panel`（状态帧快照 / panel）、`pixel_x`（像素 / X 坐标）、`done_bar_w`（完成标志 / 信息条 / W）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign info_bar_done = status_panel &&
                                 (pixel_x >= 12'd640) && (pixel_x < 12'd640 + done_bar_w) &&
                                 (pixel_y >= 12'd50) && (pixel_y < 12'd62);
          // [ASSIGN] 组合定义“info_bar_remain”：当前 pixel_x/pixel_y 对“信息 / 信息条 / 剩余量”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `status_panel`（状态帧快照 / panel）、`pixel_x`（像素 / X 坐标）、`remain_bar_w`（剩余量 / 信息条 / W）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign info_bar_remain = status_panel &&
                                   (pixel_x >= 12'd640) && (pixel_x < 12'd640 + remain_bar_w) &&
                                   (pixel_y >= 12'd70) && (pixel_y < 12'd82);
          // [ASSIGN] 组合定义“info_bar_speed”：“信息 / 信息条 / 速度”数值；在 C5 链中为轮速幅值或 11 bit 二补码有符号速度。 该组合结果直接由 `status_panel`（状态帧快照 / panel）、`pixel_x`（像素 / X 坐标）、`speed_bar_w`（速度 / 信息条 / W）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign info_bar_speed = status_panel &&
                                  (pixel_x >= 12'd640) && (pixel_x < 12'd640 + speed_bar_w) &&
                                  (pixel_y >= 12'd90) && (pixel_y < 12'd102);

          // [ASSIGN] 组合定义“car_body”：当前 pixel_x/pixel_y 对“小车 / 车体”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `pixel_x`（像素 / X 坐标）、`car_x`（小车 / X 坐标）、`pixel_y`（像素 / Y 坐标）、`car_y`（小车 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign car_body = (pixel_x >= car_x - 8) && (pixel_x <= car_x + 8) &&
                            (pixel_y >= car_y - 8) && (pixel_y <= car_y + 8);

          // [ASSIGN] 组合定义“car_arrow”：当前 pixel_x/pixel_y 对“小车 / 方向箭头”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `car_dir`（小车 / 朝向）、`DIR_UP`（DIR / UP）、`pixel_x`（像素 / X 坐标）、`car_x`（小车 / X 坐标）、`pixel_y`（像素 / Y 坐标）、`car_y`（小车 / Y 坐标）等当前值决定，不引入时钟周期延迟。
          assign car_arrow =
              (car_dir == DIR_UP)    ? ((pixel_x >= car_x - 2) && (pixel_x <= car_x + 2) &&
                                        (pixel_y >= car_y - 18) && (pixel_y <= car_y - 8)) :
              (car_dir == DIR_RIGHT) ? ((pixel_x >= car_x + 8) && (pixel_x <= car_x + 18) &&
                                        (pixel_y >= car_y - 2) && (pixel_y <= car_y + 2)) :
              (car_dir == DIR_DOWN)  ? ((pixel_x >= car_x - 2) && (pixel_x <= car_x + 2) &&
                                        (pixel_y >= car_y + 8) && (pixel_y <= car_y + 18)) :
                                       ((pixel_x >= car_x - 18) && (pixel_x <= car_x - 8) &&
                                        (pixel_y >= car_y - 2) && (pixel_y <= car_y + 2));

          // [ASSIGN] 组合定义“target_marker”：当前 pixel_x/pixel_y 对“目标 / 标记”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `pixel_x`（像素 / X 坐标）、`target_draw_x`（目标 / draw / X 坐标）、`pixel_y`（像素 / Y 坐标）、`target_draw_y`（目标 / draw / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign target_marker =
              ((pixel_x >= target_draw_x - 5) && (pixel_x <= target_draw_x + 5) &&
               (pixel_y >= target_draw_y - 1) && (pixel_y <= target_draw_y + 1)) ||
              ((pixel_x >= target_draw_x - 1) && (pixel_x <= target_draw_x + 1) &&
               (pixel_y >= target_draw_y - 5) && (pixel_y <= target_draw_y + 5));

          // [ASSIGN] 组合定义“now_marker”：当前 pixel_x/pixel_y 对“当前位置 / 标记”图形区域的命中判定；为 1 时进入对应着色分支。 该组合结果直接由 `pixel_x`（像素 / X 坐标）、`pixel_y`（像素 / Y 坐标）的当前值决定，不引入时钟周期延迟。
          assign now_marker =
              ((pixel_x >= 12'd670) && (pixel_x <= 12'd680) &&
               (pixel_y >= 12'd365) && (pixel_y <= 12'd375)) ||
              ((pixel_x >= 12'd673) && (pixel_x <= 12'd677) &&
               (pixel_y >= 12'd362) && (pixel_y <= 12'd378));

          // -----------------------------------------------------------------------------
          // [PROCESS] 过程块 1
          // 时序过程：在有效时钟边沿更新 step_pix（步骤 / PIX）、total_bar_w（总计 / 信息条 / W）、done_bar_w（完成标志 / 信息条 / W）、remain_bar_w（剩余量 / 信息条 / W）、speed_bar_w（速度 / 信息条 / W）、car_x（小车 / X 坐标）等信号；复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。
          // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
          // -----------------------------------------------------------------------------
          always @(posedge clk_pix or negedge rst_n) begin
            if (!rst_n) begin
              step_pix <= 7'd16;
              total_bar_w <= 9'd0;
              done_bar_w <= 9'd0;
              remain_bar_w <= 9'd0;
              speed_bar_w <= 9'd0;
              car_x <= 12'd675;
              car_y <= 12'd370;
              car_dir <= DIR_UP;
              target_draw_x <= 12'd675;
              target_draw_y <= 12'd370;
              stop_seg_sx <= 12'd0;
              stop_seg_sy <= 12'd0;
              stop_seg_ex <= 12'd0;
              stop_seg_ey <= 12'd0;
              stop_seg_valid <= 1'b0;
              old_seg_valid <= {ROUTE_WORDS{1'b0}};
              active_seg_valid <= {ROUTE_WORDS{1'b0}};
              for (i = 0; i < ROUTE_WORDS; i = i + 1) begin
                old_seg_sx[i] <= 12'd0;
                old_seg_sy[i] <= 12'd0;
                old_seg_ex[i] <= 12'd0;
                old_seg_ey[i] <= 12'd0;
                active_seg_sx[i] <= 12'd0;
                active_seg_sy[i] <= 12'd0;
                active_seg_ex[i] <= 12'd0;
                active_seg_ey[i] <= 12'd0;
              end
            end else begin
              step_pix <= calc_step_pix(active_route_steps);
              total_bar_w <= clamp_bar_w(total_route_length[15:3]);
              done_bar_w <= clamp_bar_w(complete_route_length[15:3]);
              remain_bar_w <= clamp_bar_w(remaining_route_length[15:3]);
              speed_bar_w <= clamp_bar_w(speed[15:1]);

              target_draw_x <= 675 + scaled_offset(target_x, calc_step_pix(active_route_steps));
              target_draw_y <= 370 + scaled_offset(target_y, calc_step_pix(active_route_steps));
              stop_seg_valid <= 1'b0;
              old_seg_valid <= {ROUTE_WORDS{1'b0}};
              active_seg_valid <= {ROUTE_WORDS{1'b0}};

              cur_x = 675;
              cur_y = 370;
              dir = DIR_UP;

              for (i = 0; i < ROUTE_WORDS; i = i + 1) begin
                old_seg_sx[i] <= 12'd0;
                old_seg_sy[i] <= 12'd0;
                old_seg_ex[i] <= 12'd0;
                old_seg_ey[i] <= 12'd0;

                cmd = old_route_info[i*2 +: 2];
                if ((cmd != ROUTE_STOP) && (i < completed_route_steps)) begin
                  if (cmd == ROUTE_RIGHT)
                    dir = dir + 2'd1;
                  else if (cmd == ROUTE_LEFT)
                    dir = dir - 2'd1;
                  else begin
                    next_x = cur_x;
                    next_y = cur_y;
                    case (dir)
                      DIR_UP:    next_y = cur_y - calc_step_pix(active_route_steps);
                      DIR_RIGHT: next_x = cur_x + calc_step_pix(active_route_steps);
                      DIR_DOWN:  next_y = cur_y + calc_step_pix(active_route_steps);
                      default:   next_x = cur_x - calc_step_pix(active_route_steps);
                    endcase
                    old_seg_valid[i] <= 1'b1;
                    old_seg_sx[i] <= cur_x[11:0];
                    old_seg_sy[i] <= cur_y[11:0];
                    old_seg_ex[i] <= next_x[11:0];
                    old_seg_ey[i] <= next_y[11:0];
                    cur_x = next_x;
                    cur_y = next_y;
                  end
                end
              end

              car_x <= cur_x[11:0];
              car_y <= cur_y[11:0];
              car_dir <= dir;

              for (i = 0; i < ROUTE_WORDS; i = i + 1) begin
                active_seg_sx[i] <= 12'd0;
                active_seg_sy[i] <= 12'd0;
                active_seg_ex[i] <= 12'd0;
                active_seg_ey[i] <= 12'd0;

                cmd = route_info[i*2 +: 2];
                if ((cmd != ROUTE_STOP) && (i < active_route_steps)) begin
                  if (cmd == ROUTE_RIGHT)
                    dir = dir + 2'd1;
                  else if (cmd == ROUTE_LEFT)
                    dir = dir - 2'd1;
                  else begin
                    next_x = cur_x;
                    next_y = cur_y;
                    case (dir)
                      DIR_UP:    next_y = cur_y - calc_step_pix(active_route_steps);
                      DIR_RIGHT: next_x = cur_x + calc_step_pix(active_route_steps);
                      DIR_DOWN:  next_y = cur_y + calc_step_pix(active_route_steps);
                      default:   next_x = cur_x - calc_step_pix(active_route_steps);
                    endcase
                    active_seg_valid[i] <= 1'b1;
                    active_seg_sx[i] <= cur_x[11:0];
                    active_seg_sy[i] <= cur_y[11:0];
                    active_seg_ex[i] <= next_x[11:0];
                    active_seg_ey[i] <= next_y[11:0];
                    cur_x = next_x;
                    cur_y = next_y;
                  end
                end
              end

              if (state_info == STATE_STOP) begin
                stop_seg_valid <= 1'b1;
                case (car_dir)
                  DIR_UP: begin
                    stop_seg_sx <= car_x - 14;
                    stop_seg_sy <= car_y - step_pix;
                    stop_seg_ex <= car_x + 14;
                    stop_seg_ey <= car_y - step_pix;
                  end
                  DIR_RIGHT: begin
                    stop_seg_sx <= car_x + step_pix;
                    stop_seg_sy <= car_y - 14;
                    stop_seg_ex <= car_x + step_pix;
                    stop_seg_ey <= car_y + 14;
                  end
                  DIR_DOWN: begin
                    stop_seg_sx <= car_x - 14;
                    stop_seg_sy <= car_y + step_pix;
                    stop_seg_ex <= car_x + 14;
                    stop_seg_ey <= car_y + step_pix;
                  end
                  default: begin
                    stop_seg_sx <= car_x - step_pix;
                    stop_seg_sy <= car_y - 14;
                    stop_seg_ex <= car_x - step_pix;
                    stop_seg_ey <= car_y + 14;
                  end
                endcase
              end
            end
          end

          // -----------------------------------------------------------------------------
          // [PROCESS] 过程块 2
          // 组合过程：根据当前输入和寄存状态计算 grid_x_line（grid / X 坐标 / 输出线）、grid_y_line（grid / Y 坐标 / 输出线）；过程内应为所有输出路径赋值，避免推断非预期锁存器。
          // -----------------------------------------------------------------------------
          always @(*) begin
            grid_x_line = (pixel_x == 12'd405) || (pixel_x == 12'd445) ||
                          (pixel_x == 12'd485) || (pixel_x == 12'd525) ||
                          (pixel_x == 12'd565) || (pixel_x == 12'd605) ||
                          (pixel_x == 12'd645) || (pixel_x == 12'd685) ||
                          (pixel_x == 12'd725) || (pixel_x == 12'd765) ||
                          (pixel_x == 12'd805) || (pixel_x == 12'd845) ||
                          (pixel_x == 12'd885) || (pixel_x == 12'd925);
            grid_y_line = (pixel_y == 12'd180) || (pixel_y == 12'd220) ||
                          (pixel_y == 12'd260) || (pixel_y == 12'd300) ||
                          (pixel_y == 12'd340) || (pixel_y == 12'd380) ||
                          (pixel_y == 12'd420) || (pixel_y == 12'd460) ||
                          (pixel_y == 12'd500) || (pixel_y == 12'd540);
          end

          // -----------------------------------------------------------------------------
          // [PROCESS] 过程块 3
          // 组合过程：根据当前输入和寄存状态计算 old_route_pixel（历史 / 路线 / 像素）、active_route_pixel（当前有效 / 路线 / 像素）、stop_line_pixel（停车 / 输出线 / 像素）；过程内应为所有输出路径赋值，避免推断非预期锁存器。
          // -----------------------------------------------------------------------------
          always @(*) begin
            old_route_pixel = 1'b0;
            active_route_pixel = 1'b0;
            stop_line_pixel = 1'b0;

            for (i = 0; i < ROUTE_WORDS; i = i + 1) begin
              if (old_seg_valid[i] &&
                  segment_hit(old_seg_sx[i], old_seg_sy[i], old_seg_ex[i], old_seg_ey[i], 1) &&
                  pixel_x[3] && pixel_y[3])
                old_route_pixel = 1'b1;
              if (active_seg_valid[i] &&
                  segment_hit(active_seg_sx[i], active_seg_sy[i], active_seg_ex[i], active_seg_ey[i], 2))
                active_route_pixel = 1'b1;
            end

            if (stop_seg_valid)
              stop_line_pixel = segment_hit(stop_seg_sx, stop_seg_sy, stop_seg_ex, stop_seg_ey, 1);
          end

          // -----------------------------------------------------------------------------
          // [PROCESS] 过程块 4
          // 组合过程：根据当前输入和寄存状态计算 txt_state0（TXT / state0）、txt_state1（TXT / state1）、txt_state2（TXT / state2）、txt_state3（TXT / state3）、txt_change_box（TXT / change / BOX）、txt_run_box（TXT / RUN / BOX）等信号；过程内应为所有输出路径赋值，避免推断非预期锁存器。
          // -----------------------------------------------------------------------------
          always @(*) begin
            txt_state0 = 1'b0;
            txt_state1 = 1'b0;
            txt_state2 = 1'b0;
            txt_state3 = 1'b0;
            txt_change_box = 1'b0;
            txt_run_box = 1'b0;
            txt_info_dark = 1'b0;
          txt_route_dark = 1'b0;
          txt_route_hex = 1'b0;
          txt_menu_hint = 1'b0;
          txt_route_hint = 1'b0;
          text_idx = 0;
            text_base_x = 0;
            text_char = " ";

            if ((pixel_y >= 12'd38) && (pixel_y < 12'd52)) begin
              if ((pixel_x >= 12'd78) && (pixel_x < 12'd124)) begin
                text_idx = slot12(pixel_x - 12'd78);
                text_base_x = 78 + (text_idx << 3) + (text_idx << 2);
                txt_state0 = glyph5x7_2x_hit(text_word_char(WORD_MENU, text_idx), pixel_x, pixel_y, text_base_x, 38);
              end
              if ((pixel_x >= 12'd220) && (pixel_x < 12'd266)) begin
                text_idx = slot12(pixel_x - 12'd220);
                text_base_x = 220 + (text_idx << 3) + (text_idx << 2);
                txt_state1 = glyph5x7_2x_hit(text_word_char(WORD_EDIT, text_idx), pixel_x, pixel_y, text_base_x, 38);
              end
              if ((pixel_x >= 12'd372) && (pixel_x < 12'd406)) begin
                text_idx = slot12(pixel_x - 12'd372);
                text_base_x = 372 + (text_idx << 3) + (text_idx << 2);
                txt_state2 = glyph5x7_2x_hit(text_word_char(WORD_RUN, text_idx), pixel_x, pixel_y, text_base_x, 38);
              end
              if ((pixel_x >= 12'd500) && (pixel_x < 12'd546)) begin
                text_idx = slot12(pixel_x - 12'd500);
                text_base_x = 500 + (text_idx << 3) + (text_idx << 2);
                txt_state3 = glyph5x7_2x_hit(text_word_char(WORD_STOP, text_idx), pixel_x, pixel_y, text_base_x, 38);
              end
              if ((pixel_x >= 12'd604) && (pixel_x < 12'd650)) begin
                text_idx = slot12(pixel_x - 12'd604);
                text_base_x = 604 + (text_idx << 3) + (text_idx << 2);
                txt_info_dark = glyph5x7_2x_hit(text_word_char(WORD_DONE, text_idx), pixel_x, pixel_y, text_base_x, 38);
              end
              if ((pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
                text_idx = slot12(pixel_x - 12'd890);
                text_base_x = 890 + (text_idx << 3) + (text_idx << 2);
                txt_info_dark = glyph5x7_2x_hit(dec5_char(complete_route_length, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 38);
              end
            end

            if ((pixel_y >= 12'd18) && (pixel_y < 12'd32)) begin
              if ((pixel_x >= 12'd598) && (pixel_x < 12'd656)) begin
                text_idx = slot12(pixel_x - 12'd598);
                text_base_x = 598 + (text_idx << 3) + (text_idx << 2);
                txt_info_dark = glyph5x7_2x_hit(text_word_char(WORD_TOTAL, text_idx), pixel_x, pixel_y, text_base_x, 18);
              end
              if ((pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
                text_idx = slot12(pixel_x - 12'd890);
                text_base_x = 890 + (text_idx << 3) + (text_idx << 2);
                txt_info_dark = glyph5x7_2x_hit(dec5_char(total_route_length, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 18);
              end
            end

            if ((pixel_y >= 12'd58) && (pixel_y < 12'd72)) begin
              if ((pixel_x >= 12'd604) && (pixel_x < 12'd650)) begin
                text_idx = slot12(pixel_x - 12'd604);
                text_base_x = 604 + (text_idx << 3) + (text_idx << 2);
                txt_info_dark = glyph5x7_2x_hit(text_word_char(WORD_LEFT, text_idx), pixel_x, pixel_y, text_base_x, 58);
              end
              if ((pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
                text_idx = slot12(pixel_x - 12'd890);
                text_base_x = 890 + (text_idx << 3) + (text_idx << 2);
                txt_info_dark = glyph5x7_2x_hit(dec5_char(remaining_route_length, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 58);
              end
            end

          if ((pixel_y >= 12'd78) && (pixel_y < 12'd92)) begin
            if ((pixel_x >= 12'd592) && (pixel_x < 12'd650)) begin
              text_idx = slot12(pixel_x - 12'd592);
                text_base_x = 592 + (text_idx << 3) + (text_idx << 2);
                txt_info_dark = glyph5x7_2x_hit(text_word_char(WORD_SPEED, text_idx), pixel_x, pixel_y, text_base_x, 78);
              end
              if ((pixel_x >= 12'd890) && (pixel_x < 12'd950)) begin
                text_idx = slot12(pixel_x - 12'd890);
                text_base_x = 890 + (text_idx << 3) + (text_idx << 2);
                txt_info_dark = glyph5x7_2x_hit(dec5_char(speed, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 78);
              end
            end

            if ((pixel_y >= 12'd260) && (pixel_y < 12'd274) &&
                (pixel_x >= 12'd112) && (pixel_x < 12'd184)) begin
              text_idx = slot12(pixel_x - 12'd112);
              text_base_x = 112 + (text_idx << 3) + (text_idx << 2);
              txt_change_box = glyph5x7_2x_hit(text_word_char(WORD_CHANGE, text_idx), pixel_x, pixel_y, text_base_x, 260);
            end

            if ((pixel_y >= 12'd390) && (pixel_y < 12'd404) &&
                (pixel_x >= 12'd164) && (pixel_x < 12'd198)) begin
              text_idx = slot12(pixel_x - 12'd164);
              text_base_x = 164 + (text_idx << 3) + (text_idx << 2);
              txt_run_box = glyph5x7_2x_hit(text_word_char(WORD_RUN, text_idx), pixel_x, pixel_y, text_base_x, 390);
            end

            if ((pixel_y >= 12'd156) && (pixel_y < 12'd170)) begin
              if ((pixel_x >= 12'd382) && (pixel_x < 12'd416)) begin
                text_idx = slot12(pixel_x - 12'd382);
                text_base_x = 382 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_ACT, text_idx), pixel_x, pixel_y, text_base_x, 156);
              end
              if ((pixel_x >= 12'd430) && (pixel_x < 12'd454)) begin
                text_idx = slot12(pixel_x - 12'd430);
                text_base_x = 430 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(dec2_char({2'b00, active_route_steps}, text_idx), pixel_x, pixel_y, text_base_x, 156);
              end
              if ((pixel_x >= 12'd478) && (pixel_x < 12'd524)) begin
                text_idx = slot12(pixel_x - 12'd478);
                text_base_x = 478 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_DONE, text_idx), pixel_x, pixel_y, text_base_x, 156);
              end
              if ((pixel_x >= 12'd538) && (pixel_x < 12'd562)) begin
                text_idx = slot12(pixel_x - 12'd538);
                text_base_x = 538 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(dec2_char({2'b00, completed_route_steps}, text_idx), pixel_x, pixel_y, text_base_x, 156);
              end
              if ((pixel_x >= 12'd586) && (pixel_x < 12'd634)) begin
                text_idx = slot12(pixel_x - 12'd586);
                text_base_x = 586 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_STEP, text_idx), pixel_x, pixel_y, text_base_x, 156);
              end
              if ((pixel_x >= 12'd658) && (pixel_x < 12'd718)) begin
                text_idx = slot12(pixel_x - 12'd658);
                text_base_x = 658 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(dec5_char(step_length, text_idx[2:0]), pixel_x, pixel_y, text_base_x, 156);
              end
              if ((pixel_x >= 12'd742) && (pixel_x < 12'd766)) begin
                text_idx = slot12(pixel_x - 12'd742);
                text_base_x = 742 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_NX, text_idx), pixel_x, pixel_y, text_base_x, 156);
              end
              if ((pixel_x >= 12'd778) && (pixel_x < 12'd814)) begin
                text_idx = slot12(pixel_x - 12'd778);
                text_base_x = 778 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(dec3_char(now_x, text_idx[1:0]), pixel_x, pixel_y, text_base_x, 156);
              end
              if ((pixel_x >= 12'd826) && (pixel_x < 12'd850)) begin
                text_idx = slot12(pixel_x - 12'd826);
                text_base_x = 826 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_NY, text_idx), pixel_x, pixel_y, text_base_x, 156);
              end
              if ((pixel_x >= 12'd862) && (pixel_x < 12'd898)) begin
                text_idx = slot12(pixel_x - 12'd862);
                text_base_x = 862 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(dec3_char(now_y, text_idx[1:0]), pixel_x, pixel_y, text_base_x, 156);
              end
            end

            if ((pixel_y >= 12'd172) && (pixel_y < 12'd186)) begin
              if ((pixel_x >= 12'd730) && (pixel_x < 12'd754)) begin
                text_idx = slot12(pixel_x - 12'd730);
                text_base_x = 730 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_TX, text_idx), pixel_x, pixel_y, text_base_x, 172);
              end
              if ((pixel_x >= 12'd766) && (pixel_x < 12'd802)) begin
                text_idx = slot12(pixel_x - 12'd766);
                text_base_x = 766 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(dec3_char(target_x, text_idx[1:0]), pixel_x, pixel_y, text_base_x, 172);
              end
              if ((pixel_x >= 12'd826) && (pixel_x < 12'd850)) begin
                text_idx = slot12(pixel_x - 12'd826);
                text_base_x = 826 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(text_word_char(WORD_TY, text_idx), pixel_x, pixel_y, text_base_x, 172);
              end
              if ((pixel_x >= 12'd862) && (pixel_x < 12'd898)) begin
                text_idx = slot12(pixel_x - 12'd862);
                text_base_x = 862 + (text_idx << 3) + (text_idx << 2);
                txt_route_dark = glyph5x7_2x_hit(dec3_char(target_y, text_idx[1:0]), pixel_x, pixel_y, text_base_x, 172);
              end
            end

            if ((pixel_y >= 12'd520) && (pixel_y < 12'd534)) begin
              if ((pixel_x >= 12'd382) && (pixel_x < 12'd406)) begin
                text_idx = slot12(pixel_x - 12'd382);
                text_base_x = 382 + (text_idx << 3) + (text_idx << 2);
                txt_route_hex = glyph5x7_2x_hit(text_word_char(WORD_RT, text_idx), pixel_x, pixel_y, text_base_x, 520);
              end
              if ((pixel_x >= 12'd418) && (pixel_x < 12'd610)) begin
                text_idx = slot12(pixel_x - 12'd418);
                text_base_x = 418 + (text_idx << 3) + (text_idx << 2);
                txt_route_hex = glyph5x7_2x_hit(hex16_char(route_info, text_idx), pixel_x, pixel_y, text_base_x, 520);
              end
            end

            if ((pixel_y >= 12'd538) && (pixel_y < 12'd552)) begin
              if ((pixel_x >= 12'd382) && (pixel_x < 12'd416)) begin
                text_idx = slot12(pixel_x - 12'd382);
                text_base_x = 382 + (text_idx << 3) + (text_idx << 2);
                txt_route_hex = glyph5x7_2x_hit(text_word_char(WORD_OLD, text_idx), pixel_x, pixel_y, text_base_x, 538);
              end
              if ((pixel_x >= 12'd430) && (pixel_x < 12'd622)) begin
                text_idx = slot12(pixel_x - 12'd430);
                text_base_x = 430 + (text_idx << 3) + (text_idx << 2);
                txt_route_hex = glyph5x7_2x_hit(hex16_char(old_route_info, text_idx), pixel_x, pixel_y, text_base_x, 538);
              end
            end

            if ((pixel_y >= 12'd458) && (pixel_y < 12'd472)) begin
              if ((pixel_x >= 12'd84) && (pixel_x < 12'd132)) begin
                text_idx = slot12(pixel_x - 12'd84);
                text_base_x = 84 + (text_idx << 3) + (text_idx << 2);
                txt_menu_hint = glyph5x7_2x_hit(text_word_char(WORD_LEFT, text_idx), pixel_x, pixel_y, text_base_x, 458);
              end
              if ((pixel_x >= 12'd156) && (pixel_x < 12'd180)) begin
                text_idx = slot12(pixel_x - 12'd156);
                text_base_x = 156 + (text_idx << 3) + (text_idx << 2);
                txt_menu_hint = glyph5x7_2x_hit(dec2_char({2'b00, active_route_steps}, text_idx), pixel_x, pixel_y, text_base_x, 458);
              end
            end

            if ((pixel_y >= 12'd478) && (pixel_y < 12'd492)) begin
              if ((pixel_x >= 12'd84) && (pixel_x < 12'd132)) begin
                text_idx = slot12(pixel_x - 12'd84);
                text_base_x = 84 + (text_idx << 3) + (text_idx << 2);
                txt_menu_hint = glyph5x7_2x_hit(text_word_char(WORD_NEXT, text_idx), pixel_x, pixel_y, text_base_x, 478);
              end
              if ((pixel_x >= 12'd156) && (pixel_x < 12'd204)) begin
                text_idx = slot12(pixel_x - 12'd156);
                text_base_x = 156 + (text_idx << 3) + (text_idx << 2);
                txt_menu_hint = glyph5x7_2x_hit(cmd_word_char(route_info[1:0], text_idx), pixel_x, pixel_y, text_base_x, 478);
              end
            end

            if ((pixel_y >= 12'd194) && (pixel_y < 12'd208)) begin
              if ((pixel_x >= 12'd382) && (pixel_x < 12'd418)) begin
                text_idx = slot12(pixel_x - 12'd382);
                text_base_x = 382 + (text_idx << 3) + (text_idx << 2);
                txt_route_hint = glyph5x7_2x_hit(text_word_char(WORD_OLD, text_idx), pixel_x, pixel_y, text_base_x, 194);
              end
              if ((pixel_x >= 12'd450) && (pixel_x < 12'd498)) begin
                text_idx = slot12(pixel_x - 12'd450);
                text_base_x = 450 + (text_idx << 3) + (text_idx << 2);
                txt_route_hint = glyph5x7_2x_hit(text_word_char(WORD_PATH, text_idx), pixel_x, pixel_y, text_base_x, 194);
              end
              if ((pixel_x >= 12'd542) && (pixel_x < 12'd578)) begin
                text_idx = slot12(pixel_x - 12'd542);
                text_base_x = 542 + (text_idx << 3) + (text_idx << 2);
                txt_route_hint = glyph5x7_2x_hit(text_word_char(WORD_ACT, text_idx), pixel_x, pixel_y, text_base_x, 194);
              end
              if ((pixel_x >= 12'd578) && (pixel_x < 12'd626)) begin
                text_idx = slot12(pixel_x - 12'd578);
                text_base_x = 578 + (text_idx << 3) + (text_idx << 2);
                txt_route_hint = glyph5x7_2x_hit(text_word_char(WORD_PATH, text_idx), pixel_x, pixel_y, text_base_x, 194);
              end
            end
          end

          // -----------------------------------------------------------------------------
          // [PROCESS] 过程块 5
          // 组合过程：根据当前输入和寄存状态计算 rgb_next（RGB 像素 / 下一值）；过程内应为所有输出路径赋值，避免推断非预期锁存器。
          // -----------------------------------------------------------------------------
          always @(*) begin
            if (!pixel_de) begin
              rgb_next = 24'h000000;
            end else begin
              rgb_next = 24'hF4F7FB;

              if (status_panel) rgb_next = 24'hFFFFFF;
              if (menu_panel) rgb_next = 24'hEAF2FF;
              if (route_panel) rgb_next = 24'hF8FAFC;
              if (map_area) rgb_next = 24'hFFFFFF;
              if (grid_line) rgb_next = 24'hE5E7EB;

              if (change_box) rgb_next = change_select ? 24'h1D4ED8 : 24'hDCEAFE;
              if (run_box) rgb_next = run_select ? 24'h16A34A : 24'hDCFCE7;

              if (state_bar_0) rgb_next = state_menu_sel   ? 24'h2563EB : 24'hCBD5E1;
              if (state_bar_1) rgb_next = state_change_sel ? 24'h2563EB : 24'hCBD5E1;
              if (state_bar_2) rgb_next = state_run_sel    ? 24'h16A34A : 24'hCBD5E1;
              if (state_bar_3) rgb_next = state_stop_sel   ? 24'hDC2626 : 24'hCBD5E1;
              if (bt_status_box) rgb_next = bt_connected ? 24'hDCFCE7 : 24'hE5E7EB;

              if (info_bar_total) rgb_next = 24'h2563EB;
              if (info_bar_done) rgb_next = 24'h16A34A;
              if (info_bar_remain) rgb_next = 24'hF59E0B;
              if (info_bar_speed) rgb_next = 24'h7C3AED;

              if (txt_info_dark || txt_route_dark || txt_route_hex || txt_menu_hint || txt_route_hint)
                rgb_next = 24'h0F172A;
              if (txt_state0) rgb_next = state_menu_sel ? 24'hFFFFFF : 24'h334155;
              if (txt_state1) rgb_next = state_change_sel ? 24'hFFFFFF : 24'h334155;
              if (txt_state2) rgb_next = state_run_sel ? 24'hFFFFFF : 24'h334155;
            if (txt_state3) rgb_next = state_stop_sel ? 24'hFFFFFF : 24'h334155;
            if (txt_change_box) rgb_next = change_select ? 24'hFFFFFF : 24'h1D4ED8;
            if (txt_run_box) rgb_next = run_select ? 24'hFFFFFF : 24'h166534;

            if (old_route_pixel) rgb_next = 24'h94A3B8;
              if (active_route_pixel) rgb_next = 24'h2563EB;
              if (target_marker) rgb_next = 24'h0891B2;
              if (now_marker) rgb_next = 24'h64748B;
              if (car_body) rgb_next = 24'h111827;
              if (car_arrow) rgb_next = 24'hFACC15;
              if (stop_line_pixel) rgb_next = 24'hDC2626;
              if (border_line) rgb_next = 24'h334155;
            end
          end

          // -----------------------------------------------------------------------------
          // [PROCESS] 过程块 6
          // 时序过程：在有效时钟边沿更新 rgb_data（RGB 像素 / 数据）；复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。
          // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
          // -----------------------------------------------------------------------------
          always @(posedge clk_pix or negedge rst_n) begin
            if (!rst_n)
              rgb_data <= 24'h000000;
            else
              rgb_data <= rgb_next;
          end

endmodule
