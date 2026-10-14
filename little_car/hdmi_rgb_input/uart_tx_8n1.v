module uart_tx_8n1(
input wire clk,
input wire rst_n,
input wire start,
input wire [7:0] data,
output reg tx,
output reg busy
);

  localparam integer CLK_HZ = 50000000;
  localparam integer BAUD = 115200;
  localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;

  localparam [1:0] STATE_IDLE  = 2'd0;
  localparam [1:0] STATE_START = 2'd1;
  localparam [1:0] STATE_DATA  = 2'd2;
  localparam [1:0] STATE_STOP  = 2'd3;

  reg [1:0] state;
  reg [15:0] clk_count;
  reg [2:0] bit_index;
  reg [7:0] shifter;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx <= 1'b1;
      busy <= 1'b0;
      state <= STATE_IDLE;
      clk_count <= 16'd0;
      bit_index <= 3'd0;
      shifter <= 8'd0;
    end else begin
      case (state)
        STATE_IDLE: begin
          tx <= 1'b1;
          busy <= 1'b0;
          clk_count <= 16'd0;
          bit_index <= 3'd0;
          if (start) begin
            busy <= 1'b1;
            shifter <= data;
            state <= STATE_START;
            tx <= 1'b0;
          end
        end

        STATE_START: begin
          busy <= 1'b1;
          tx <= 1'b0;
          if (clk_count == CLKS_PER_BIT - 1) begin
            clk_count <= 16'd0;
            state <= STATE_DATA;
            tx <= shifter[0];
          end else begin
            clk_count <= clk_count + 16'd1;
          end
        end

        STATE_DATA: begin
          busy <= 1'b1;
          tx <= shifter[bit_index];
          if (clk_count == CLKS_PER_BIT - 1) begin
            clk_count <= 16'd0;
            if (bit_index == 3'd7) begin
              bit_index <= 3'd0;
              state <= STATE_STOP;
              tx <= 1'b1;
            end else begin
              bit_index <= bit_index + 3'd1;
              tx <= shifter[bit_index + 3'd1];
            end
          end else begin
            clk_count <= clk_count + 16'd1;
          end
        end

        default: begin
          busy <= 1'b1;
          tx <= 1'b1;
          if (clk_count == CLKS_PER_BIT - 1) begin
            clk_count <= 16'd0;
            busy <= 1'b0;
            state <= STATE_IDLE;
          end else begin
            clk_count <= clk_count + 16'd1;
          end
        end
      endcase
    end
  end

endmodule
