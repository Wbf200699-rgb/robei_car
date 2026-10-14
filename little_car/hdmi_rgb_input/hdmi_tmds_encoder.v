module hdmi_tmds_encoder(
input wire clk_pix,
input wire rst_n,
input wire [7:0] data_in,
input wire c0,
input wire c1,
input wire v_de,
output reg [9:0] tmds_10b
);

  //----Code starts here: integrated by Robei-----
  wire [3:0] n1d = data_in[0] + data_in[1] + data_in[2] + data_in[3] +
                   data_in[4] + data_in[5] + data_in[6] + data_in[7];

  wire [8:0] q_m;
  wire use_xnor = (n1d > 4'd4) || (n1d == 4'd4 && data_in[0] == 1'b0);

  assign q_m[0] = data_in[0];
  assign q_m[1] = use_xnor ? (q_m[0] ^~ data_in[1]) : (q_m[0] ^ data_in[1]);
  assign q_m[2] = use_xnor ? (q_m[1] ^~ data_in[2]) : (q_m[1] ^ data_in[2]);
  assign q_m[3] = use_xnor ? (q_m[2] ^~ data_in[3]) : (q_m[2] ^ data_in[3]);
  assign q_m[4] = use_xnor ? (q_m[3] ^~ data_in[4]) : (q_m[3] ^ data_in[4]);
  assign q_m[5] = use_xnor ? (q_m[4] ^~ data_in[5]) : (q_m[4] ^ data_in[5]);
  assign q_m[6] = use_xnor ? (q_m[5] ^~ data_in[6]) : (q_m[5] ^ data_in[6]);
  assign q_m[7] = use_xnor ? (q_m[6] ^~ data_in[7]) : (q_m[6] ^ data_in[7]);
  assign q_m[8] = use_xnor ? 1'b0 : 1'b1;

  reg  [3:0] cnt;
  wire [3:0] n1q_m = q_m[0] + q_m[1] + q_m[2] + q_m[3] +
                     q_m[4] + q_m[5] + q_m[6] + q_m[7];
  wire [3:0] n0q_m = 4'd8 - n1q_m;

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
endmodule    //hdmi_tmds_encoder
