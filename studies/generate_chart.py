"""
generate_chart.py — Study Results Visualiser
---------------------------------------------
Reads studies/results.csv and produces:
  1. Time distribution histogram + KDE
  2. CPU & RAM usage over run index
  3. Scatter: wall time vs simulation time

Usage:
    python studies/generate_chart.py [--input studies/results.csv]

Dependencies:
    pip install pandas matplotlib seaborn
"""

import argparse
import os
import sys

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

try:
    import seaborn as sns
except ImportError:
    print("ERROR: seaborn is required.  Install with: pip install seaborn")
    sys.exit(1)

# ── Anduril-inspired colour palette ──────────────────────────────────────────
ANDURIL_BG     = "#1E2124"
ANDURIL_PANEL  = "#282B30"
ANDURIL_GREEN  = "#00FF41"
ANDURIL_CYAN   = "#00D4FF"
ANDURIL_AMBER  = "#FFB700"
ANDURIL_RED    = "#FF3366"
ANDURIL_TEXT   = "#E0E5EA"
ANDURIL_GRID   = "#3E4348"

plt.rcParams.update({
    "figure.facecolor":  ANDURIL_BG,
    "axes.facecolor":    ANDURIL_PANEL,
    "axes.edgecolor":    ANDURIL_GRID,
    "axes.labelcolor":   ANDURIL_TEXT,
    "text.color":        ANDURIL_TEXT,
    "xtick.color":       ANDURIL_TEXT,
    "ytick.color":       ANDURIL_TEXT,
    "grid.color":        ANDURIL_GRID,
    "grid.linestyle":    "--",
    "grid.alpha":        0.4,
    "font.family":       "monospace",
})

def load(path: str) -> pd.DataFrame:
    if not os.path.exists(path):
        print(f"ERROR: {path} not found. Run the orchestrator first.")
        sys.exit(1)
    df = pd.read_csv(path)

    # normalise column names — older CSVs may lack cpu/ram columns
    for col in ["internal_time_s", "wall_time_s", "cpu_pct", "ram_pct"]:
        if col not in df.columns:
            df[col] = float("nan")

    df["internal_time_s"] = pd.to_numeric(df["internal_time_s"], errors="coerce")
    df["wall_time_s"]     = pd.to_numeric(df["wall_time_s"],     errors="coerce")
    df["cpu_pct"]         = pd.to_numeric(df["cpu_pct"],         errors="coerce")
    df["ram_pct"]         = pd.to_numeric(df["ram_pct"],         errors="coerce")

    # keep only successful runs for timing charts
    df_ok = df[df["internal_time_s"] >= 0].copy()
    return df, df_ok


def generate_charts(input_path: str, output_dir: str):
    df, df_ok = load(input_path)

    total_runs   = len(df)
    success_runs = len(df_ok)
    mean_t       = df_ok["internal_time_s"].mean()
    median_t     = df_ok["internal_time_s"].median()
    min_t        = df_ok["internal_time_s"].min()
    max_t        = df_ok["internal_time_s"].max()

    has_resources = df["cpu_pct"].notna().any()

    # ── Figure layout ─────────────────────────────────────────────────────
    n_rows = 3 if has_resources else 1
    fig = plt.figure(figsize=(14, 5 * n_rows), tight_layout=True)
    fig.suptitle(
        f"Swarm AI Exploration Study  ·  {success_runs}/{total_runs} runs succeeded",
        fontsize=15, fontweight="bold", color=ANDURIL_TEXT, y=1.01
    )
    gs = gridspec.GridSpec(n_rows, 2, figure=fig, hspace=0.45, wspace=0.35)

    # ── 1. Time distribution (histogram + KDE) ───────────────────────────
    ax1 = fig.add_subplot(gs[0, :])
    sns.histplot(
        df_ok["internal_time_s"],
        kde=True,
        color=ANDURIL_GREEN,
        edgecolor=ANDURIL_BG,
        alpha=0.55,
        linewidth=1.2,
        ax=ax1,
    )
    if ax1.lines:
        ax1.lines[0].set(color=ANDURIL_GREEN, linewidth=2.5)

    ax1.axvline(mean_t,   color=ANDURIL_RED,   linestyle="--", linewidth=1.8,
                label=f"Mean   {mean_t:.2f}s")
    ax1.axvline(median_t, color=ANDURIL_AMBER, linestyle=":",  linewidth=1.8,
                label=f"Median {median_t:.2f}s")
    ax1.set_title("Simulation Time to 95% Explored", fontsize=13, pad=8)
    ax1.set_xlabel("Simulation Time (s)")
    ax1.set_ylabel("Frequency")
    ax1.legend(facecolor=ANDURIL_PANEL, edgecolor=ANDURIL_GRID, labelcolor=ANDURIL_TEXT)

    stats_text = (
        f"n={success_runs}   min={min_t:.1f}s   max={max_t:.1f}s   "
        f"mean={mean_t:.2f}s   median={median_t:.2f}s"
    )
    ax1.text(0.01, 0.96, stats_text, transform=ax1.transAxes,
             fontsize=9, va="top", color=ANDURIL_TEXT, alpha=0.8)

    if has_resources:
        run_idx = range(len(df))

        # ── 2. CPU usage over run index ───────────────────────────────────
        ax2 = fig.add_subplot(gs[1, 0])
        ax2.plot(run_idx, df["cpu_pct"], color=ANDURIL_CYAN,  linewidth=0.8, alpha=0.75)
        ax2.axhline(60, color=ANDURIL_RED,  linestyle="--", linewidth=1.2, label="60% limit")
        ax2.fill_between(run_idx, df["cpu_pct"], alpha=0.15, color=ANDURIL_CYAN)
        ax2.set_ylim(0, max(df["cpu_pct"].max() * 1.2, 70))
        ax2.set_title("CPU Usage per Run", fontsize=11)
        ax2.set_xlabel("Run Index")
        ax2.set_ylabel("CPU (%)")
        ax2.legend(facecolor=ANDURIL_PANEL, edgecolor=ANDURIL_GRID, labelcolor=ANDURIL_TEXT, fontsize=8)

        # ── 3. RAM usage over run index ───────────────────────────────────
        ax3 = fig.add_subplot(gs[1, 1])
        ax3.plot(run_idx, df["ram_pct"], color=ANDURIL_AMBER, linewidth=0.8, alpha=0.75)
        ax3.axhline(60, color=ANDURIL_RED,  linestyle="--", linewidth=1.2, label="60% limit")
        ax3.fill_between(run_idx, df["ram_pct"], alpha=0.15, color=ANDURIL_AMBER)
        ax3.set_ylim(0, max(df["ram_pct"].max() * 1.2, 70))
        ax3.set_title("RAM Usage per Run", fontsize=11)
        ax3.set_xlabel("Run Index")
        ax3.set_ylabel("RAM (%)")
        ax3.legend(facecolor=ANDURIL_PANEL, edgecolor=ANDURIL_GRID, labelcolor=ANDURIL_TEXT, fontsize=8)

        # ── 4. Scatter: wall time vs sim time (coloured by CPU) ───────────
        ax4 = fig.add_subplot(gs[2, :])
        sc = ax4.scatter(
            df_ok["wall_time_s"], df_ok["internal_time_s"],
            c=df_ok["cpu_pct"] if df_ok["cpu_pct"].notna().any() else ANDURIL_GREEN,
            cmap="YlGn",
            s=18, alpha=0.7, edgecolors="none"
        )
        cbar = plt.colorbar(sc, ax=ax4)
        cbar.set_label("CPU % at End of Run", color=ANDURIL_TEXT)
        cbar.ax.yaxis.set_tick_params(color=ANDURIL_TEXT)
        plt.setp(plt.getp(cbar.ax.axes, "yticklabels"), color=ANDURIL_TEXT)
        ax4.set_title("Wall Time vs Simulation Time (colour = CPU%)", fontsize=11)
        ax4.set_xlabel("Wall Clock Time (s)")
        ax4.set_ylabel("Simulation Time (s)")

    # ── Save ──────────────────────────────────────────────────────────────
    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, "time_distribution.png")
    fig.savefig(out_path, dpi=200, facecolor=fig.get_facecolor(), bbox_inches="tight")
    print(f"✅  Chart saved to {out_path}")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Generate study result charts")
    parser.add_argument("--input",  default="studies/results.csv",    help="CSV results file")
    parser.add_argument("--output", default="studies",                help="Output directory")
    args = parser.parse_args()
    generate_charts(args.input, args.output)


if __name__ == "__main__":
    main()
