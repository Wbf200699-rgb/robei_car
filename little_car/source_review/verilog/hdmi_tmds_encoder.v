// =============================================================================
// 模块: hdmi_tmds_encoder
// 作用: 单颜色通道 TMDS 8 bit 到 10 bit 编码器。
// 说明: 消隐期输出控制码；有效期根据输入 1 的数量和 running disparity 选择 XOR/XNOR 与反相。
//
// Robei 源: little_car/model/hdmi_tmds_encoder.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module hdmi_tmds_encoder (
  input wire clk_pix,  // HDMI 像素时钟域时钟。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire [7:0] data_in,  // 当前颜色通道待编码的 8 bit 像素字节。
  input wire c0,  // 消隐期 TMDS 控制位 0；蓝色通道连接 hsync。
  input wire c1,  // 消隐期 TMDS 控制位 1；蓝色通道连接 vsync。
  input wire v_de,  // 数据有效选择；1 编码像素数据，0 输出四种控制码之一。
  output reg [9:0] tmds_10b  // 按 running disparity 平衡后的 10 bit TMDS 码字。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
// -----------------------------------------------------------------------------
// 内部网络、寄存器和计数器
// wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
// 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
// -----------------------------------------------------------------------------
wire [3:0] n1d = data_in[0] + data_in[1] + data_in[2] + data_in[3] +  // 原始输入字节中逻辑 1 的数量。
                 data_in[4] + data_in[5] + data_in[6] + data_in[7];

wire [8:0] q_m;  // 第一阶段 8 bit XOR/XNOR 转换结果。
wire use_xnor = (n1d > 4'd4) || (n1d == 4'd4 && data_in[0] == 1'b0);  // 输入 1 的数量较多时选择 XNOR 链，减少第一阶段码字翻转次数。

// -----------------------------------------------------------------------------
// 组合派生信号
// 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
// -----------------------------------------------------------------------------
// [ASSIGN] 组合定义“q_m”：第一阶段 8 bit XOR/XNOR 转换结果。 该组合结果直接由 `data_in`（数据 / IN）的当前值决定，不引入时钟周期延迟。
assign q_m[0] = data_in[0];
// [ASSIGN] 组合定义“q_m”：第一阶段 8 bit XOR/XNOR 转换结果。 该组合结果直接由 `use_xnor`（USE / xnor）、`q_m`（Q / M）、`data_in`（数据 / IN）的当前值决定，不引入时钟周期延迟。
assign q_m[1] = use_xnor ? (q_m[0] ^~ data_in[1]) : (q_m[0] ^ data_in[1]);
// [ASSIGN] 组合定义“q_m”：第一阶段 8 bit XOR/XNOR 转换结果。 该组合结果直接由 `use_xnor`（USE / xnor）、`q_m`（Q / M）、`data_in`（数据 / IN）的当前值决定，不引入时钟周期延迟。
assign q_m[2] = use_xnor ? (q_m[1] ^~ data_in[2]) : (q_m[1] ^ data_in[2]);
// [ASSIGN] 组合定义“q_m”：第一阶段 8 bit XOR/XNOR 转换结果。 该组合结果直接由 `use_xnor`（USE / xnor）、`q_m`（Q / M）、`data_in`（数据 / IN）的当前值决定，不引入时钟周期延迟。
assign q_m[3] = use_xnor ? (q_m[2] ^~ data_in[3]) : (q_m[2] ^ data_in[3]);
// [ASSIGN] 组合定义“q_m”：第一阶段 8 bit XOR/XNOR 转换结果。 该组合结果直接由 `use_xnor`（USE / xnor）、`q_m`（Q / M）、`data_in`（数据 / IN）的当前值决定，不引入时钟周期延迟。
assign q_m[4] = use_xnor ? (q_m[3] ^~ data_in[4]) : (q_m[3] ^ data_in[4]);
// [ASSIGN] 组合定义“q_m”：第一阶段 8 bit XOR/XNOR 转换结果。 该组合结果直接由 `use_xnor`（USE / xnor）、`q_m`（Q / M）、`data_in`（数据 / IN）的当前值决定，不引入时钟周期延迟。
assign q_m[5] = use_xnor ? (q_m[4] ^~ data_in[5]) : (q_m[4] ^ data_in[5]);
// [ASSIGN] 组合定义“q_m”：第一阶段 8 bit XOR/XNOR 转换结果。 该组合结果直接由 `use_xnor`（USE / xnor）、`q_m`（Q / M）、`data_in`（数据 / IN）的当前值决定，不引入时钟周期延迟。
assign q_m[6] = use_xnor ? (q_m[5] ^~ data_in[6]) : (q_m[5] ^ data_in[6]);
// [ASSIGN] 组合定义“q_m”：第一阶段 8 bit XOR/XNOR 转换结果。 该组合结果直接由 `use_xnor`（USE / xnor）、`q_m`（Q / M）、`data_in`（数据 / IN）的当前值决定，不引入时钟周期延迟。
assign q_m[7] = use_xnor ? (q_m[6] ^~ data_in[7]) : (q_m[6] ^ data_in[7]);
// [ASSIGN] 组合定义“q_m”：第一阶段 8 bit XOR/XNOR 转换结果。 该组合结果直接由 `use_xnor`（USE / xnor）的当前值决定，不引入时钟周期延迟。
assign q_m[8] = use_xnor ? 1'b0 : 1'b1;

reg  [3:0] cnt;  // TMDS 当前码流的 4 bit 有符号 running disparity 累计值；消隐时清零，有效像素时决定是否反相。
wire [3:0] n1q_m = q_m[0] + q_m[1] + q_m[2] + q_m[3] +  // q_m[7:0] 中逻辑 1 的数量。
                   q_m[4] + q_m[5] + q_m[6] + q_m[7];
wire [3:0] n0q_m = 4'd8 - n1q_m;  // q_m[7:0] 中逻辑 0 的数量，由 8-n1q_m 得到。

// -----------------------------------------------------------------------------
// [PROCESS] 过程块 1
// TMDS 编码时序：消隐期输出控制码；有效期更新 10 bit 码和 running disparity。
// 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
// -----------------------------------------------------------------------------
always @(posedge clk_pix or negedge rst_n) begin
    if (!rst_n) begin
        tmds_10b <= 10'b0;
        cnt      <= 4'b0;

    end else if (!v_de) begin
        cnt <= 4'b0;
        case ({c1, c0})
            2'b00:  tmds_10b <= 10'b1101010100;
            2'b01:  tmds_10b <= 10'b0010101011;
            2'b10:  tmds_10b <= 10'b0101010100;
            default:tmds_10b <= 10'b1010101011;
        endcase

    end else begin
        if (cnt == 4'd0 || n1q_m == n0q_m) begin
            tmds_10b <= {~q_m[8], q_m[8],
                         (q_m[8] ? q_m[7:0] : ~q_m[7:0])};

            if (q_m[8] == 1'b0)
                cnt <= cnt + (n0q_m - n1q_m);
            else
                cnt <= cnt + (n1q_m - n0q_m);

        end else begin
            if ((cnt[3] == 1'b0 && n1q_m > n0q_m) ||
                (cnt[3] == 1'b1 && n0q_m > n1q_m)) begin

                tmds_10b <= {1'b1, q_m[8], ~q_m[7:0]};
                cnt <= cnt + {q_m[8], 1'b0} + (n0q_m - n1q_m);

            end else begin
                tmds_10b <= {1'b0, q_m[8], q_m[7:0]};
                cnt <= cnt - {~q_m[8], 1'b0} + (n1q_m - n0q_m);
            end
        end
    end
end

endmodule
