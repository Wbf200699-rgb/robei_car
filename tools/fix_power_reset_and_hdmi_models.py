from pathlib import Path
import re
import shutil
import time


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "little_car" / "model"
BUILD_DIR = ROOT / "little_car" / "build"

RESET_NET = "sys_route_gui_fsm_top_rst_n"

CLK_WIZ_BODY = """  `ifdef SYNTHESIS
    wire clkfb_mmcm;
    wire clkfb_bufg;
    wire clk_pix_mmcm;
    wire clk_5x_mmcm;
    wire mmcm_locked;

    MMCME2_ADV #(
      .BANDWIDTH("OPTIMIZED"),
      .CLKIN1_PERIOD(20.000),
      .CLKIN2_PERIOD(0.000),
      .CLKFBOUT_MULT_F(20.000),
      .CLKFBOUT_PHASE(0.000),
      .DIVCLK_DIVIDE(1),
      .CLKOUT0_DIVIDE_F(20.000),
      .CLKOUT0_PHASE(0.000),
      .CLKOUT0_DUTY_CYCLE(0.500),
      .CLKOUT1_DIVIDE(4),
      .CLKOUT1_PHASE(0.000),
      .CLKOUT1_DUTY_CYCLE(0.500),
      .CLKOUT2_DIVIDE(16),
      .CLKOUT2_PHASE(0.000),
      .CLKOUT2_DUTY_CYCLE(0.500),
      .CLKOUT3_DIVIDE(16),
      .CLKOUT3_PHASE(0.000),
      .CLKOUT3_DUTY_CYCLE(0.500),
      .CLKOUT4_DIVIDE(16),
      .CLKOUT4_PHASE(0.000),
      .CLKOUT4_DUTY_CYCLE(0.500),
      .CLKOUT5_DIVIDE(16),
      .CLKOUT5_PHASE(0.000),
      .CLKOUT5_DUTY_CYCLE(0.500),
      .CLKOUT6_DIVIDE(16),
      .CLKOUT6_PHASE(0.000),
      .CLKOUT6_DUTY_CYCLE(0.500),
      .COMPENSATION("ZHOLD"),
      .REF_JITTER1(0.010),
      .STARTUP_WAIT("FALSE")
    ) mmcm_inst (
      .CLKIN1(clk_in1),
      .CLKIN2(1'b0),
      .CLKINSEL(1'b1),
      .RST(~resetn),
      .PWRDWN(1'b0),
      .CLKFBIN(clkfb_bufg),
      .CLKFBOUT(clkfb_mmcm),
      .CLKFBOUTB(),
        .CLKOUT0(clk_pix_mmcm),
        .CLKOUT0B(),
        .CLKOUT1(clk_5x_mmcm),
        .CLKOUT1B(),
      .CLKOUT2(),
      .CLKOUT2B(),
      .CLKOUT3(),
      .CLKOUT3B(),
      .CLKOUT4(),
      .CLKOUT5(),
      .CLKOUT6(),
      .DO(),
      .DRDY(),
      .DADDR(7'd0),
      .DCLK(1'b0),
      .DEN(1'b0),
      .DI(16'd0),
      .DWE(1'b0),
      .PSCLK(1'b0),
      .PSEN(1'b0),
      .PSINCDEC(1'b0),
      .PSDONE(),
      .LOCKED(mmcm_locked),
      .CLKINSTOPPED(),
      .CLKFBSTOPPED()
    );

    BUFG clkfb_bufg_inst (
      .I(clkfb_mmcm),
      .O(clkfb_bufg)
    );

    BUFG clk_pix_bufg_inst (
      .I(clk_pix_mmcm),
      .O(clk_out1)
    );

    BUFG clk_5x_bufg_inst (
      .I(clk_5x_mmcm),
      .O(clk_out2)
    );

    assign locked = mmcm_locked;
  `else
    reg clk_50m = 1'b0;
    reg clk_250m = 1'b0;
    reg is_locked = 1'b0;

    always @(posedge clk_in1 or negedge resetn) begin
      if (!resetn)
        is_locked <= 1'b0;
      else
        #100 is_locked <= 1'b1;
    end

    always #10.000 clk_50m = ~clk_50m;
    always #2.000 clk_250m = ~clk_250m;

    assign clk_out1 = clk_50m;
    assign clk_out2 = clk_250m;
    assign locked = is_locked;
  `endif"""


def encode_robei_attr(value: str) -> str:
    value = value.replace("\r\n", "\n").replace("\r", "\n")
    value = value.replace("&", "&amp;")
    value = value.replace("<", "&lt;")
    value = value.replace(">", "&gt;")
    value = value.replace('"', "&quot;")
    return value.replace("\n", "&#xa;")


def backup(path: Path) -> None:
    if path.exists():
        stamp = time.strftime("%Y%m%d_%H%M%S")
        shutil.copy2(path, path.with_name(path.name + f".bak_{stamp}"))


def replace_module_code(path: Path, instance_name: str, code: str) -> int:
    text = path.read_text(encoding="utf-8")
    encoded = encode_robei_attr(code)
    changed = 0

    def replace_tag(match: re.Match) -> str:
        nonlocal changed
        tag = match.group(0)
        if f'Name="{instance_name}"' not in tag:
            return tag
        new_tag, count = re.subn(r'Code="[^"]*"', f'Code="{encoded}"', tag, count=1)
        changed += count
        return new_tag

    new_text = re.sub(r"<Module\b[^>]*>", replace_tag, text)
    if changed:
        backup(path)
        path.write_text(new_text, encoding="utf-8", newline="\n")
    return changed


def fix_top_model_reset_fanout(path: Path) -> int:
    text = path.read_text(encoding="utf-8")
    pattern = re.compile(
        r'(<Wire\b'
        r'(?=[^>]*From="sys_route_gui_fsm_top#power_on_reset24&gt;system_rst_n")'
        r'(?=[^>]*To="sys_route_gui_fsm_top#[^"]+&gt;rst_n")'
        r'[^>]*\bName=")[^"]+(")'
    )
    new_text, count = pattern.subn(rf'\1{RESET_NET}\2', text)
    if count:
        backup(path)
        path.write_text(new_text, encoding="utf-8", newline="\n")
    return count


def render_clk_wiz_build(path: Path) -> None:
    text = """module clk_wiz_0(
input wire clk_in1,
input wire resetn,
output wire clk_out1,
output wire clk_out2,
output wire locked
);

  //----Code starts here: integrated by Robei-----
""" + CLK_WIZ_BODY + """
endmodule    //clk_wiz_0
"""
    backup(path)
    path.write_text(text, encoding="utf-8", newline="\n")


def fix_build_top_reset(path: Path) -> int:
    text = path.read_text(encoding="utf-8")
    text, count_out = re.subn(
        r"\.system_rst_n\(sys_route_gui_fsm_top_rst_n_[A-Za-z0-9_]+\)",
        f".system_rst_n({RESET_NET})",
        text,
    )
    text, count_in = re.subn(
        r"\.rst_n\(sys_route_gui_fsm_top_rst_n_[A-Za-z0-9_]+\)",
        f".rst_n({RESET_NET})",
        text,
    )
    count = count_out + count_in
    if count:
        backup(path)
        path.write_text(text, encoding="utf-8", newline="\n")
    return count


def main() -> None:
    changed = {
        "top reset wires": fix_top_model_reset_fanout(MODEL_DIR / "sys_route_gui_fsm_top.model"),
        "clk_wiz_0.model": replace_module_code(MODEL_DIR / "clk_wiz_0.model", "clk_wiz_0", CLK_WIZ_BODY),
        "hdmi_top.model child": replace_module_code(MODEL_DIR / "hdmi_top.model", "clk_wiz_01", CLK_WIZ_BODY),
        "build top reset refs": fix_build_top_reset(BUILD_DIR / "sys_route_gui_fsm_top.v"),
    }
    render_clk_wiz_build(BUILD_DIR / "clk_wiz_0.v")
    for key, value in changed.items():
        print(f"{key}: {value}")
    print("build clk_wiz_0.v: rewritten")


if __name__ == "__main__":
    main()
