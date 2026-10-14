// =============================================================================
// 模块: gui_route_test_top
// 作用: 独立 HDMI GUI 演示顶层。
// 说明: 使用临时状态源验证显示，不属于当前小车系统顶层。
//
// Robei 源: little_car/model/gui_route_test_top.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module gui_route_test_top (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  output wire hdmi_clk_p,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_clk_n,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d0_p,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d0_n,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d1_p,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d2_p,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d2_n,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d1_n  // HDMI TMDS 差分物理输出端。
);

  // ===========================================================================
  // 层次内部网络
  // 每条注释给出驱动端以及所有接收端，便于从顶层沿数据流追踪。
  // ===========================================================================
  // hdmi_top2.pixel_x -> gui_route_renderer1.pixel_x
  wire [11:0] hdmi_top2_pixel_x;  // 当前有效显示区水平像素坐标。 具体驱动端和全部接收端见上一行。
  // hdmi_top2.pixel_y -> gui_route_renderer1.pixel_y
  wire [11:0] hdmi_top2_pixel_y;  // 当前有效显示区垂直像素坐标。 具体驱动端和全部接收端见上一行。
  // gui_route_renderer1.rgb_data -> hdmi_top2.rgb_data
  wire [23:0] gui_route_renderer1_rgb_data;  // GUI 输出或 HDMI 输入的 RGB888 像素。 具体驱动端和全部接收端见上一行。
  // hdmi_top2.pixel_de -> gui_route_renderer1.pixel_de
  wire hdmi_top2_pixel_de;  // 视频数据有效；1 表示位于可见区域。 具体驱动端和全部接收端见上一行。
  // gui_route_demo_info_source3.route_info -> gui_route_renderer1.route_info
  wire [63:0] gui_route_demo_info_source3_route_info;  // 64 bit 路线命令队列；最低 2 bit 是下一条命令。 具体驱动端和全部接收端见上一行。
  // hdmi_top2.pixel_clk -> gui_route_renderer1.clk_pix
  wire hdmi_top2_pixel_clk;  // 导出给 GUI 渲染器使用的 50 MHz HDMI 像素时钟。 具体驱动端和全部接收端见上一行。
  // gui_route_demo_info_source3.system_info -> gui_route_renderer1.menu_info
  wire [1:0] gui_route_demo_info_source3_system_info;  // GUI 演示使用的固定 2 bit 系统状态，不进入实际小车主链。 具体驱动端和全部接收端见上一行。
  // gui_menu_to_state_info4.menu_info -> gui_route_renderer1.menu_info
  wire [1:0] gui_menu_to_state_info4_menu_info;  // 菜单焦点/页面状态编码。 具体驱动端和全部接收端见上一行。
  // gui_menu_to_state_info4.state_info -> gui_route_renderer1.state_info
  wire [4:0] gui_menu_to_state_info4_state_info;  // 路线 FSM 状态编码。 具体驱动端和全部接收端见上一行。
  // temp_gui_status_source5.old_route_info -> gui_route_renderer1.old_route_info
  wire [63:0] temp_gui_status_source5_old_route_info;  // 已完成路线历史，用于 GUI 和手机显示。 具体驱动端和全部接收端见上一行。
  // temp_gui_status_source5.active_route_steps -> gui_route_renderer1.active_route_steps
  wire [5:0] temp_gui_status_source5_active_route_steps;  // 当前待执行内部命令数量。 具体驱动端和全部接收端见上一行。
  // temp_gui_status_source5.completed_route_steps -> gui_route_renderer1.completed_route_steps
  wire [5:0] temp_gui_status_source5_completed_route_steps;  // 已完成内部命令数量。 具体驱动端和全部接收端见上一行。
  // temp_gui_status_source5.step_length -> gui_route_renderer1.step_length
  wire [15:0] temp_gui_status_source5_step_length;  // 当前生效的路线单步长度；设置为零时路线核心回退到默认值 20。 具体驱动端和全部接收端见上一行。
  // temp_gui_status_source5.speed -> gui_route_renderer1.speed
  wire [15:0] temp_gui_status_source5_speed;  // 当前生效的路线整体速度参数，向 GUI 和执行控制链同步。 具体驱动端和全部接收端见上一行。
  // temp_gui_status_source5.total_route_length -> gui_route_renderer1.total_route_length
  wire [15:0] temp_gui_status_source5_total_route_length;  // 已完成步数与待执行步数之和乘以 step_length 得到的计划总长度。 具体驱动端和全部接收端见上一行。
  // temp_gui_status_source5.complete_route_length -> gui_route_renderer1.complete_route_length
  wire [15:0] temp_gui_status_source5_complete_route_length;  // completed_route_steps 乘以 step_length 得到的已完成路线长度。 具体驱动端和全部接收端见上一行。
  // temp_gui_status_source5.remaining_route_length -> gui_route_renderer1.remaining_route_length
  wire [15:0] temp_gui_status_source5_remaining_route_length;  // active_route_steps 乘以 step_length 得到的剩余路线长度。 具体驱动端和全部接收端见上一行。
  // temp_gui_status_source5.now_x -> gui_route_renderer1.now_x
  wire [7:0] temp_gui_status_source5_now_x;  // 小车当前逻辑栅格 X 坐标；自动动作完成后由路线核心更新。 具体驱动端和全部接收端见上一行。
  // temp_gui_status_source5.now_y -> gui_route_renderer1.now_y
  wire [7:0] temp_gui_status_source5_now_y;  // 小车当前逻辑栅格 Y 坐标；自动动作完成后由路线核心更新。 具体驱动端和全部接收端见上一行。
  // temp_gui_status_source5.target_x -> gui_route_renderer1.target_x
  wire [7:0] temp_gui_status_source5_target_x;  // 编辑路线末端预测 X 坐标；添加或撤销动作时同步变化。 具体驱动端和全部接收端见上一行。
  // temp_gui_status_source5.target_y -> gui_route_renderer1.target_y
  wire [7:0] temp_gui_status_source5_target_y;  // 编辑路线末端预测 Y 坐标；添加或撤销动作时同步变化。 具体驱动端和全部接收端见上一行。

  // ---------------------------------------------------------------------------
  // 子模块实例: gui_route_renderer1
  // 类型: gui_route_renderer
  // 作用: 按像素坐标组合绘制小车状态、菜单、路线网格、当前位置和目标。
  // 连接说明: 该模块不保存帧缓存，每个像素直接由坐标、路线和状态组合计算 RGB。
  // 连接说明: 路线命令被展开为水平/垂直线段；字符使用 5x7 点阵放大两倍绘制。
  // ---------------------------------------------------------------------------
  gui_route_renderer
  gui_route_renderer1 (
    .pixel_x(hdmi_top2_pixel_x),
    .pixel_y(hdmi_top2_pixel_y),
    .pixel_de(hdmi_top2_pixel_de),
    .route_info(gui_route_demo_info_source3_route_info),
    .rgb_data(gui_route_renderer1_rgb_data),
    .clk_pix(hdmi_top2_pixel_clk),
    .rst_n(rst_n),
    .menu_info(gui_menu_to_state_info4_menu_info),
    .state_info(gui_menu_to_state_info4_state_info),
    .old_route_info(temp_gui_status_source5_old_route_info),
    .active_route_steps(temp_gui_status_source5_active_route_steps),
    .completed_route_steps(temp_gui_status_source5_completed_route_steps),
    .step_length(temp_gui_status_source5_step_length),
    .speed(temp_gui_status_source5_speed),
    .total_route_length(temp_gui_status_source5_total_route_length),
    .complete_route_length(temp_gui_status_source5_complete_route_length),
    .remaining_route_length(temp_gui_status_source5_remaining_route_length),
    .now_x(temp_gui_status_source5_now_x),
    .now_y(temp_gui_status_source5_now_y),
    .target_x(temp_gui_status_source5_target_x),
    .target_y(temp_gui_status_source5_target_y)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: hdmi_top2
  // 类型: hdmi_top
  // 作用: 完整 HDMI 发送流水线的层次封装。
  // 连接说明: 时钟 -> 1024x600 时序 -> RGB 拆分 -> 三路 TMDS 编码 -> 四路串化差分输出。
  // 连接说明: 蓝色通道承载 hsync/vsync 控制码，红绿通道消隐期控制输入固定为 0。
  // ---------------------------------------------------------------------------
  hdmi_top
  hdmi_top2 (
    .clk(clk),
    .rst_n(rst_n),
    .hdmi_clk_p(hdmi_clk_p),
    .hdmi_d0_p(hdmi_d0_p),
    .hdmi_d1_p(hdmi_d1_p),
    .hdmi_d2_p(hdmi_d2_p),
    .hdmi_clk_n(hdmi_clk_n),
    .hdmi_d0_n(hdmi_d0_n),
    .hdmi_d1_n(hdmi_d1_n),
    .hdmi_d2_n(hdmi_d2_n),
    .rgb_data(gui_route_renderer1_rgb_data),
    .pixel_x(hdmi_top2_pixel_x),
    .pixel_y(hdmi_top2_pixel_y),
    .pixel_de(hdmi_top2_pixel_de),
    .pixel_clk(hdmi_top2_pixel_clk)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: gui_route_demo_info_source3
  // 类型: gui_route_demo_info_source
  // 作用: GUI 演示路线与状态常量源。
  // 连接说明: 只用于显示测试。
  // ---------------------------------------------------------------------------
  gui_route_demo_info_source
  gui_route_demo_info_source3 (
    .system_info(gui_route_demo_info_source3_system_info),
    .route_info(gui_route_demo_info_source3_route_info)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: gui_menu_to_state_info4
  // 类型: gui_menu_to_state_info
  // 作用: 把简化菜单编码映射成 GUI 状态编码。
  // 连接说明: 只用于演示顶层。
  // ---------------------------------------------------------------------------
  gui_menu_to_state_info
  gui_menu_to_state_info4 (
    .menu_info(gui_menu_to_state_info4_menu_info),
    .state_info(gui_menu_to_state_info4_state_info)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: temp_gui_status_source5
  // 类型: temp_gui_status_source
  // 作用: GUI 临时状态源。
  // 连接说明: 为早期显示验证提供固定运行状态。
  // ---------------------------------------------------------------------------
  temp_gui_status_source
  temp_gui_status_source5 (
    .old_route_info(temp_gui_status_source5_old_route_info),
    .active_route_steps(temp_gui_status_source5_active_route_steps),
    .completed_route_steps(temp_gui_status_source5_completed_route_steps),
    .step_length(temp_gui_status_source5_step_length),
    .speed(temp_gui_status_source5_speed),
    .total_route_length(temp_gui_status_source5_total_route_length),
    .complete_route_length(temp_gui_status_source5_complete_route_length),
    .remaining_route_length(temp_gui_status_source5_remaining_route_length),
    .now_x(temp_gui_status_source5_now_x),
    .now_y(temp_gui_status_source5_now_y),
    .target_x(temp_gui_status_source5_target_x),
    .target_y(temp_gui_status_source5_target_y)
  );

endmodule
