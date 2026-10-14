module menu_to_state_info(
input wire [1:0] menu_info,
output reg [4:0] state_info
);

  always @(*) begin
    case (menu_info)
      2'b01: state_info = 5'b00001;
      2'b11: state_info = 5'b00011;
      default: state_info = 5'b00000;
    endcase
  end

endmodule
