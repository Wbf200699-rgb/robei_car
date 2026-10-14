from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle


OUT_DIR = Path(__file__).resolve().parent
STEM = OUT_DIR / "path_control_main_fsm"

COLORS = {
    "ink": "#17232D",
    "muted": "#61717C",
    "line": "#AAB6BE",
    "panel": "#F5F7F8",
    "blue": "#245B78",
    "blue_fill": "#E4F0F5",
    "teal": "#3F8D8A",
    "teal_fill": "#E1F1EF",
    "green": "#58866B",
    "green_fill": "#E8F1EA",
    "coral": "#C85E55",
    "coral_fill": "#F8E8E5",
    "gold": "#A97A2E",
    "gold_fill": "#F7F0DE",
    "white": "#FFFFFF",
}


mpl.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Microsoft YaHei", "Arial", "DejaVu Sans", "sans-serif"],
        "font.size": 7,
        "text.color": COLORS["ink"],
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "axes.linewidth": 0.8,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
    }
)


def box(ax, xy, width, height, face, edge, radius=0.012, lw=0.9, z=2):
    patch = FancyBboxPatch(
        xy,
        width,
        height,
        boxstyle=f"round,pad=0.006,rounding_size={radius}",
        facecolor=face,
        edgecolor=edge,
        linewidth=lw,
        zorder=z,
    )
    ax.add_patch(patch)
    return patch


def arrow(
    ax,
    start,
    end,
    color=None,
    lw=1.25,
    style="-|>",
    mutation=9,
    connection="arc3,rad=0",
    linestyle="-",
    z=4,
):
    patch = FancyArrowPatch(
        start,
        end,
        arrowstyle=style,
        mutation_scale=mutation,
        linewidth=lw,
        color=color or COLORS["blue"],
        connectionstyle=connection,
        linestyle=linestyle,
        shrinkA=0,
        shrinkB=0,
        zorder=z,
    )
    ax.add_patch(patch)
    return patch


def label(ax, x, y, text, size=7, weight="normal", color=None, ha="left", va="center", **kwargs):
    return ax.text(
        x,
        y,
        text,
        fontsize=size,
        fontweight=weight,
        color=color or COLORS["ink"],
        ha=ha,
        va=va,
        zorder=8,
        **kwargs,
    )


def state_box(ax, x, y, w, h, code, title, subtitle, fill, edge):
    box(ax, (x, y), w, h, fill, edge, radius=0.015, lw=1.15)
    label(ax, x + 0.014, y + h - 0.021, code, size=5.8, weight="bold", color=edge)
    label(ax, x + w / 2, y + h * 0.53, title, size=10, weight="bold", ha="center")
    label(ax, x + w / 2, y + 0.021, subtitle, size=5.8, color=COLORS["muted"], ha="center")


def make_figure():
    # 183 mm two-column width, compact schematic-led composition.
    fig = plt.figure(figsize=(7.205, 4.50))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    # Header and conclusion line.
    label(ax, 0.035, 0.956, "路径控制主状态机", size=13.5, weight="bold")
    label(
        ax,
        0.035,
        0.918,
        "统一输入驱动同一编码、撤销与执行逻辑，使路径内容、位姿和进度保持同步",
        size=7.1,
        color=COLORS["muted"],
    )
    ax.plot([0.035, 0.965], [0.892, 0.892], color=COLORS["line"], lw=0.65, zorder=1)

    # Section headings.
    label(ax, 0.035, 0.861, "a", size=8.5, weight="bold")
    label(ax, 0.057, 0.861, "统一事件入口", size=7.8, weight="bold")
    label(ax, 0.222, 0.861, "b", size=8.5, weight="bold")
    label(ax, 0.244, 0.861, "主状态机与原子更新", size=7.8, weight="bold")
    label(ax, 0.794, 0.861, "c", size=8.5, weight="bold")
    label(ax, 0.816, 0.861, "同步状态视图", size=7.8, weight="bold")

    # Left: unified physical/mobile event source.
    box(ax, (0.035, 0.697), 0.145, 0.105, COLORS["blue_fill"], COLORS["blue"], radius=0.012)
    label(ax, 0.108, 0.765, "车端物理按键", size=7.4, weight="bold", ha="center")
    label(ax, 0.108, 0.728, "↑  ↓  ←  →  确认", size=7.0, color=COLORS["blue"], ha="center")

    box(ax, (0.035, 0.546), 0.145, 0.105, COLORS["teal_fill"], COLORS["teal"], radius=0.012)
    label(ax, 0.108, 0.614, "Android APP", size=7.4, weight="bold", ha="center")
    label(ax, 0.108, 0.577, "路径编辑命令", size=6.6, color=COLORS["teal"], ha="center")

    # Merge spine.
    arrow(ax, (0.180, 0.749), (0.207, 0.749), COLORS["blue"], mutation=8)
    arrow(ax, (0.180, 0.598), (0.207, 0.598), COLORS["teal"], mutation=8)
    ax.plot([0.207, 0.207], [0.598, 0.749], color=COLORS["line"], lw=1.1, zorder=2)
    arrow(ax, (0.207, 0.674), (0.245, 0.674), COLORS["blue"], mutation=9)
    label(ax, 0.108, 0.490, "虚拟脉冲与物理脉冲合并", size=6.1, color=COLORS["muted"], ha="center")
    label(ax, 0.108, 0.462, "key_*_pulse", size=5.8, color=COLORS["blue"], ha="center", family="monospace")

    # Hero state-machine plate.
    box(ax, (0.235, 0.405), 0.53, 0.397, COLORS["panel"], "#D2DADE", radius=0.010, lw=0.8, z=1)

    state_box(ax, 0.270, 0.659, 0.127, 0.102, "STATE_MENU", "菜单 / 等待", "选择编辑或运行", COLORS["white"], COLORS["blue"])
    state_box(ax, 0.452, 0.650, 0.147, 0.120, "STATE_CHANGE", "路径编辑", "追加、组合标记与撤销", COLORS["teal_fill"], COLORS["teal"])
    state_box(ax, 0.629, 0.659, 0.105, 0.102, "STATE_RUN", "自动执行", "逐字取指", COLORS["green_fill"], COLORS["green"])

    # Main transitions.
    arrow(ax, (0.397, 0.727), (0.452, 0.727), COLORS["teal"], mutation=8)
    label(ax, 0.424, 0.746, "进入编辑", size=5.3, color=COLORS["teal"], ha="center")
    arrow(ax, (0.452, 0.685), (0.397, 0.685), COLORS["muted"], mutation=8)
    label(ax, 0.424, 0.668, "确认", size=5.3, color=COLORS["muted"], ha="center")
    arrow(ax, (0.397, 0.701), (0.629, 0.701), COLORS["green"], mutation=8, connection="arc3,rad=-0.15")
    label(ax, 0.514, 0.778, "启动 · active > 0", size=5.3, color=COLORS["green"], ha="center")
    arrow(ax, (0.629, 0.679), (0.397, 0.679), COLORS["muted"], mutation=8, connection="arc3,rad=-0.18")
    label(ax, 0.519, 0.622, "active = 0", size=5.3, color=COLORS["muted"], ha="center")

    # Edit logic cards.
    box(ax, (0.264, 0.452), 0.140, 0.125, COLORS["white"], COLORS["line"], radius=0.008, lw=0.75)
    label(ax, 0.276, 0.554, "普通前进", size=6.6, weight="bold", color=COLORS["blue"])
    label(ax, 0.276, 0.522, "追加  11", size=6.4)
    label(ax, 0.276, 0.492, "占 1 个指令字", size=5.8, color=COLORS["muted"])
    label(ax, 0.276, 0.466, "step_is_turn = 0", size=5.3, color=COLORS["muted"], family="monospace")

    box(ax, (0.417, 0.452), 0.164, 0.125, COLORS["white"], COLORS["line"], radius=0.008, lw=0.75)
    label(ax, 0.429, 0.554, "左 / 右转组合", size=6.6, weight="bold", color=COLORS["teal"])
    label(ax, 0.429, 0.522, "10|11   或   01|11", size=6.1)
    label(ax, 0.429, 0.492, "连续占 2 个指令字", size=5.8, color=COLORS["muted"])
    label(ax, 0.429, 0.466, "step_is_turn = 1", size=5.3, color=COLORS["muted"], family="monospace")

    box(ax, (0.594, 0.452), 0.142, 0.125, COLORS["coral_fill"], COLORS["coral"], radius=0.008, lw=0.85)
    label(ax, 0.606, 0.554, "逻辑撤销", size=6.6, weight="bold", color=COLORS["coral"])
    label(ax, 0.606, 0.522, "普通：回退 1 字", size=5.9)
    label(ax, 0.606, 0.493, "转向：整体回退 2 字", size=5.9)
    label(ax, 0.606, 0.466, "恢复 target 位姿", size=5.3, color=COLORS["coral"])

    # Internal relationships inside edit state.
    arrow(ax, (0.526, 0.650), (0.500, 0.580), COLORS["teal"], mutation=7, connection="arc3,rad=0.12")
    arrow(ax, (0.578, 0.650), (0.665, 0.580), COLORS["coral"], mutation=7, connection="arc3,rad=-0.12", linestyle="--")

    # Execution feedback rail and stopped state.
    box(ax, (0.270, 0.331), 0.464, 0.052, COLORS["green_fill"], COLORS["green"], radius=0.010, lw=0.9)
    label(ax, 0.284, 0.357, "run_step_pulse", size=6.2, weight="bold", color=COLORS["green"], family="monospace")
    label(ax, 0.402, 0.357, "history 写入 · completed++ · current 位姿更新 · 队列左移 · active--", size=5.2)
    ax.plot([0.734, 0.750, 0.750], [0.357, 0.357, 0.625], color=COLORS["green"], lw=1.15, zorder=3)
    arrow(ax, (0.750, 0.625), (0.704, 0.659), COLORS["green"], mutation=8)

    # Right synchronized state bank.
    box(ax, (0.794, 0.683), 0.171, 0.119, COLORS["blue_fill"], COLORS["blue"], radius=0.010, lw=0.85)
    label(ax, 0.809, 0.777, "路径寄存器", size=7.0, weight="bold", color=COLORS["blue"])
    label(ax, 0.809, 0.744, "route_info / old_route_info", size=5.6, family="monospace")
    label(ax, 0.809, 0.715, "active / completed steps", size=5.6, family="monospace")
    label(ax, 0.948, 0.777, "同步", size=5.3, color=COLORS["blue"], ha="right")

    box(ax, (0.794, 0.535), 0.171, 0.119, COLORS["teal_fill"], COLORS["teal"], radius=0.010, lw=0.85)
    label(ax, 0.809, 0.629, "位姿寄存器", size=7.0, weight="bold", color=COLORS["teal"])
    label(ax, 0.809, 0.596, "current:  x, y, direction", size=5.6, family="monospace")
    label(ax, 0.809, 0.567, "target:   x, y, direction", size=5.6, family="monospace")

    box(ax, (0.794, 0.387), 0.171, 0.119, COLORS["gold_fill"], COLORS["gold"], radius=0.010, lw=0.85)
    label(ax, 0.809, 0.481, "统一状态输出", size=7.0, weight="bold", color=COLORS["gold"])
    label(ax, 0.809, 0.448, "HDMI 路径与执行进度", size=5.8)
    label(ax, 0.809, 0.419, "Android 路线重建", size=5.8)

    # Fan-out from hero to state bank.
    arrow(ax, (0.765, 0.702), (0.794, 0.742), COLORS["blue"], mutation=7)
    arrow(ax, (0.765, 0.620), (0.794, 0.594), COLORS["teal"], mutation=7)
    arrow(ax, (0.765, 0.510), (0.794, 0.446), COLORS["gold"], mutation=7)

    # Bottom instruction memory strip.
    label(ax, 0.035, 0.288, "2 bit 指令存储 · 16 个有效指令字", size=7.5, weight="bold")
    label(ax, 0.965, 0.288, "活动路径按执行顺序左移", size=5.7, color=COLORS["muted"], ha="right")

    codes = ["11", "10", "11", "01", "11", "11", "00", "00", "00", "00", "00", "00", "00", "00", "00", "00"]
    code_color = {"00": "#E5E8EA", "01": "#F1D8D3", "10": "#DDEAE5", "11": "#DCE9F2"}
    edge_color = {"00": "#B9C1C6", "01": COLORS["coral"], "10": COLORS["green"], "11": COLORS["blue"]}
    x0, y0, gap, width, height = 0.035, 0.205, 0.004, 0.054, 0.057
    for idx, code in enumerate(codes):
        x = x0 + idx * (width + gap)
        ax.add_patch(Rectangle((x, y0), width, height, facecolor=code_color[code], edgecolor=edge_color[code], linewidth=0.75, zorder=2))
        label(ax, x + width / 2, y0 + height * 0.58, code, size=7.2, weight="bold", color=edge_color[code], ha="center", family="monospace")
        label(ax, x + width / 2, y0 - 0.016, str(idx), size=4.7, color=COLORS["muted"], ha="center")
    arrow(ax, (0.410, 0.179), (0.115, 0.179), COLORS["green"], lw=0.9, mutation=7)
    label(ax, 0.421, 0.179, "run_step_pulse 后移出首字", size=5.3, color=COLORS["green"])

    # Compact direct key, avoiding a detached legend box.
    legend_y = 0.105
    legend = [
        ("00", "停止 / 空指令", edge_color["00"]),
        ("01", "右转", edge_color["01"]),
        ("10", "左转", edge_color["10"]),
        ("11", "前进", edge_color["11"]),
    ]
    xs = [0.035, 0.245, 0.405, 0.565]
    for x, (code, text, color) in zip(xs, legend):
        label(ax, x, legend_y, code, size=6.8, weight="bold", color=color, family="monospace")
        label(ax, x + 0.037, legend_y, text, size=5.8, color=COLORS["muted"])
    label(ax, 0.965, legend_y, "实线：正向状态流    虚线：撤销 / 停止反馈", size=5.2, color=COLORS["muted"], ha="right")

    label(
        ax,
        0.035,
        0.050,
        "关键约束  每次逻辑输入与实际指令字占用解耦；所有路径、坐标、方向和进度更新在同一状态机中完成。",
        size=5.8,
        color=COLORS["muted"],
    )

    return fig


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig = make_figure()
    fig.savefig(f"{STEM}.svg", bbox_inches="tight", pad_inches=0.02)
    fig.savefig(f"{STEM}.pdf", bbox_inches="tight", pad_inches=0.02)
    fig.savefig(f"{STEM}.png", dpi=600, bbox_inches="tight", pad_inches=0.02)
    fig.savefig(f"{STEM}.tiff", dpi=600, bbox_inches="tight", pad_inches=0.02, pil_kwargs={"compression": "tiff_lzw"})
    plt.close(fig)


if __name__ == "__main__":
    main()
