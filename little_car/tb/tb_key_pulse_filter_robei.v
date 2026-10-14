module key_pulse_filter_test();
reg clk;
reg rst_n;
reg key_n;
wire key_pulse;

  //----Code starts here: integrated by Robei-----
  initial
  begin
    clk = 1'b0;
    forever #10 clk = ~clk;
  end

  initial
  begin
    rst_n = 1'b0;
    key_n = 1'b1;

    #200;
    rst_n = 1'b1;

    #1000;
    key_n = 1'b0; #2000;
    key_n = 1'b1; #2000;
    key_n = 1'b0; #4000;
    key_n = 1'b1; #20000;

    key_n = 1'b0;
    #25000000;
    key_n = 1'b1;
    #5000000;

    key_n = 1'b0;
    #23000000;
    key_n = 1'b1;
    #5000000;

    $finish;
  end

  initial begin
    $dumpfile("tb/tb_key_pulse_filter_robei.vcd");
    $dumpvars(0, key_pulse_filter_test);
  end
//---Module instantiation---
  key_pulse_filter key_pulse_filter0(
    .clk(clk),
    .rst_n(rst_n),
    .key_n(key_n),
    .key_pulse(key_pulse)
);

endmodule    //key_pulse_filter_test
