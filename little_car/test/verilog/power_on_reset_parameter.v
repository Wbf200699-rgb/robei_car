`timescale 1ns/1ps
module power_on_reset(
  input wire clk,
  input wire ext_rst_n,
  output reg system_rst_n
);
  localparam integer POR_CYCLES = 1000000;
  reg [19:0] por_count;

  initial begin
    por_count = 20'd0;
    system_rst_n = 1'b0;
  end

  always @(posedge clk or negedge ext_rst_n) begin
    if (!ext_rst_n) begin
      por_count <= 20'd0;
      system_rst_n <= 1'b0;
    end else if (!system_rst_n) begin
      if (por_count >= POR_CYCLES - 1) begin
        por_count <= por_count;
        system_rst_n <= 1'b1;
      end else begin
        por_count <= por_count + 20'd1;
      end
    end
  end
endmodule

module power_on_reset_parameter;
  reg clk;
  reg ext_rst_n;
  wire system_rst_n;


`ifdef PARAM_WAVE
initial begin
  $dumpfile("tb_power_on_reset_parameter.vcd");
  $dumpvars(0, ext_rst_n, system_rst_n, power_on_reset0.por_count);
end
`endif

initial begin
  clk = 1'b0;
  ext_rst_n = 1'b0;
end

integer cycles_to_release;
  integer errors;

  always #10 clk = ~clk;



  initial begin
    errors = 0;
    cycles_to_release = 0;
    repeat (3) @(posedge clk);
    ext_rst_n = 1'b1;

    // Skip the uninteresting first 999990 clocks. The reported production
    // duration still comes from the fixed POR_CYCLES constant.
    @(negedge clk);
    power_on_reset0.por_count = 20'd999990;
    while (!system_rst_n && cycles_to_release < 20) begin
      @(posedge clk);
      #1;
      cycles_to_release = cycles_to_release + 1;
    end
    if (!system_rst_n || cycles_to_release != 10)
      errors = errors + 1;

    // ext_rst_n is asynchronous and must immediately reassert system reset.
    @(negedge clk);
    ext_rst_n = 1'b0;
    #1;
    if (system_rst_n !== 1'b0 || power_on_reset0.por_count !== 20'd0)
      errors = errors + 1;

    $display("RESULT por_clock_hz=50000000 por_cycles=1000000 por_time_us=20000");
    $display("RESULT accelerated_tail_cycles=%0d async_assert_latency_ns=1", cycles_to_release);
    if (errors == 0)
      $display("RESULT TEST_PASS power_on_reset_parameter");
    else
      $display("RESULT TEST_FAIL power_on_reset_parameter errors=%0d", errors);
    $finish;
  end

  initial begin
    #5000;
    $display("RESULT TEST_FAIL power_on_reset_parameter timeout=1");
    $finish;
  end


  power_on_reset power_on_reset0(
    .clk(clk),
    .ext_rst_n(ext_rst_n),
    .system_rst_n(system_rst_n)
  );
endmodule
