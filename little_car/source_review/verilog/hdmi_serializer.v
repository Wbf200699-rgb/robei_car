// =============================================================================
// 模块: hdmi_serializer
// 作用: 把一个像素周期的 10 bit TMDS 字并串转换为差分输出。
// 说明: 250 MHz 双沿 ODDR 实现等效 500 Mbit/s；OBUFDS 使用 TMDS_33 标准。
//
// Robei 源: little_car/model/hdmi_serializer.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module hdmi_serializer (
  input wire clk_pix,  // HDMI 像素时钟域时钟。
  input wire clk_5x,  // 像素时钟 5 倍频，用于 TMDS 双沿串化。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire [9:0] data_10b,  // 像素域待串化的单通道 10 bit TMDS 码字。
  output wire ser_p,  // OBUFDS 输出的 TMDS 正端串行信号。
  output wire ser_n  // OBUFDS 输出的 TMDS 负端串行信号。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
    // -----------------------------------------------------------------------------
    // 内部网络、寄存器和计数器
    // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
    // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
    // -----------------------------------------------------------------------------
    reg [9:0] pix_word;  // 像素时钟域锁存的完整 10 bit TMDS 码字；锁存后由 pix_toggle 通知 250 MHz 域取走。
    reg pix_toggle;  // 像素域每接收一个新 TMDS 码字翻转一次的事件位；以电平翻转方式跨时钟域传递“新字到达”。
    reg pix_toggle_sync0;  // pix_toggle 进入 250 MHz 域后的第一级同步采样，用于降低跨时钟域亚稳态传播风险。
    reg pix_toggle_sync1;  // pix_toggle 的第二级同步采样；与 sync2 不同时表示检测到一次新的像素码字。
    reg pix_toggle_sync2;  // pix_toggle_sync1 的上一周期值；和 sync1 异或形成单周期 load_word_5x。
    reg [9:0] shift_word;  // 250 MHz 域正在串化的 10 bit TMDS 移位寄存器；每周期右移两位供 ODDR 双沿输出。
    reg [2:0] pair_idx;  // 当前 10 bit TMDS 字已输出的双 bit 组编号 0..4；五个 clk_5x 周期发送完一个字。
    reg ser_d1;  // 当前 clk_5x 上升沿由 ODDR 输出的 TMDS 串行 bit。
    reg ser_d2;  // 当前 clk_5x 下降沿由 ODDR 输出的 TMDS 串行 bit。
    reg [1:0] reset_sync_5x;  // 把异步低有效复位同步释放到 250 MHz 域的两级移位寄存器；bit1 形成 rst_5x_n。
    wire load_word_5x;  // 像素 toggle 两级同步值不相等时产生的新 10 bit 字装载脉冲。
    wire rst_5x_n;  // 进入 250 MHz 域并同步释放后的低有效复位。
    (* keep = "true" *) wire ser_data_out;  // ODDR 产生、尚未经过差分缓冲的单端 TMDS 串行码流；随后由 OBUFDS 转成 ser_p/ser_n。

    // -----------------------------------------------------------------------------
    // [PROCESS] 过程块 1
    // 像素域锁存新的 10 bit TMDS 字，并通过 toggle 通知 5 倍频时钟域。
    // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
    // -----------------------------------------------------------------------------
    always @(posedge clk_pix or negedge rst_n) begin
      if (!rst_n) begin
        pix_word <= 10'd0;
        pix_toggle <= 1'b0;
      end else begin
        pix_word <= data_10b;
        pix_toggle <= ~pix_toggle;
      end
    end

    // -----------------------------------------------------------------------------
    // [PROCESS] 过程块 2
    // 跨时钟域同步 toggle，形成一次 load_pulse。
    // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
    // -----------------------------------------------------------------------------
    always @(posedge clk_5x or negedge rst_n) begin
      if (!rst_n)
        reset_sync_5x <= 2'b00;
      else
        reset_sync_5x <= {reset_sync_5x[0], 1'b1};
    end

    // -----------------------------------------------------------------------------
    // [PROCESS] 过程块 3
    // 5 倍频域每周期输出两位给 ODDR，五周期发送完整 10 bit。
    // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
    // -----------------------------------------------------------------------------
    always @(posedge clk_5x or negedge rst_5x_n) begin
      if (!rst_5x_n) begin
        pix_toggle_sync0 <= 1'b0;
        pix_toggle_sync1 <= 1'b0;
        pix_toggle_sync2 <= 1'b0;
        shift_word <= 10'd0;
        pair_idx <= 3'd0;
        ser_d1 <= 1'b0;
        ser_d2 <= 1'b0;
      end else begin
        pix_toggle_sync0 <= pix_toggle;
        pix_toggle_sync1 <= pix_toggle_sync0;
        pix_toggle_sync2 <= pix_toggle_sync1;

        if (load_word_5x) begin
          shift_word <= pix_word;
          pair_idx <= 3'd0;
          ser_d1 <= pix_word[0];
          ser_d2 <= pix_word[1];
        end else begin
          shift_word <= {2'b00, shift_word[9:2]};
          if (pair_idx < 3'd4)
            pair_idx <= pair_idx + 3'd1;
          ser_d1 <= shift_word[2];
          ser_d2 <= shift_word[3];
        end
      end
    end

    // -----------------------------------------------------------------------------
    // 组合派生信号
    // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
    // -----------------------------------------------------------------------------
    // [ASSIGN] 组合定义“load_word_5x”：像素 toggle 两级同步值不相等时产生的新 10 bit 字装载脉冲。 该组合结果直接由 `pix_toggle_sync1`（PIX / toggle / sync1）、`pix_toggle_sync2`（PIX / toggle / sync2）的当前值决定，不引入时钟周期延迟。
    assign load_word_5x = pix_toggle_sync1 ^ pix_toggle_sync2;
    // [ASSIGN] 组合定义“rst_5x_n”：进入 250 MHz 域并同步释放后的低有效复位。 该组合结果直接由 `reset_sync_5x`（reset / 同步值 / 5X）的当前值决定，不引入时钟周期延迟。
    assign rst_5x_n = reset_sync_5x[1];

    (* keep = "true", dont_touch = "true" *)
    ODDR #(
      .DDR_CLK_EDGE("SAME_EDGE"),
      .INIT(1'b0),
      .SRTYPE("SYNC")
    ) oddr_inst (
      .Q(ser_data_out),
      .C(clk_5x),
      .CE(1'b1),
      .D1(ser_d1),
      .D2(ser_d2),
      .R(1'b0),
      .S(1'b0)
    );

    (* keep = "true", dont_touch = "true" *)
    OBUFDS #(
      .IOSTANDARD("TMDS_33")
    ) obuf_inst (
      .I  (ser_data_out),
      .O  (ser_p),
      .OB (ser_n)
    );

endmodule
