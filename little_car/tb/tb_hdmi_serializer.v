module hdmi_serializer_test();
reg clk_pix;
reg clk_5x;
reg rst_n;
reg [9:0] data_10b;
wire ser_p;
wire ser_n;

  //----Code starts here: integrated by Robei-----
  initial
  begin
    clk_pix = 1'b0;
    forever #10 clk_pix = ~clk_pix;
  end

  initial
  begin
    clk_5x = 1'b0;
    forever #2 clk_5x = ~clk_5x;
  end

  task load_word;
    input [9:0] word;
    begin
      @(negedge clk_pix);
      data_10b = word;
      @(posedge clk_pix);
      #1;
    end
  endtask

  initial
  begin
    rst_n = 1'b0;
    data_10b = 10'b0000000000;

    #100;
    rst_n = 1'b1;

    load_word(10'b1100110011);
    repeat (6) @(posedge clk_pix);

    load_word(10'b1111100000);
    repeat (6) @(posedge clk_pix);

    load_word(10'b1010101010);
    repeat (6) @(posedge clk_pix);

    load_word(10'b0000011111);
    repeat (6) @(posedge clk_pix);

    load_word(10'b0101010101);
    repeat (10) @(posedge clk_pix);

    $finish;
  end

  initial begin
    $dumpfile("tb/tb_hdmi_serializer.vcd");
    $dumpvars(0, hdmi_serializer_test);
  end
//---Module instantiation---
  hdmi_serializer hdmi_serializer0(
    .clk_pix(clk_pix),
    .clk_5x(clk_5x),
    .rst_n(rst_n),
    .data_10b(data_10b),
    .ser_p(ser_p),
    .ser_n(ser_n)
);

endmodule    //hdmi_serializer_test
