"""
orchestrator.py — Drone Sim Study Orchestrator (Python)
--------------------------------------------------------
Runs headless Drone_Sim experiments ONE AT A TIME (sequential).
Simple, safe, and easy on your machine — no parallel spawning.

Usage:
    python orchestrator.py [--runs N] [--seed-start N] [--timeout N] [--output PATH]

Dependencies:
    pip install psutil
"""

import subprocess
import time
import sys
import os
import csv
import argparse
import threading

# Force UTF-8 output on Windows so box-drawing characters don't crash cp1252 terminals
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

try:
    import psutil
except ImportError:
    print("ERROR: psutil is required.  Install it with:  pip install psutil")
    sys.exit(1)

# ── ANSI colour helpers ───────────────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

def bar(pct: float, width: int = 20) -> str:
    filled = int(pct / 100 * width)
    colour = GREEN if pct < 50 else (YELLOW if pct < 75 else RED)
    return colour + "█" * filled + "░" * (width - filled) + RESET

# ── Resource sampler (runs on a background thread) ───────────────────────────
class ResourceMonitor:
    """Continuously samples CPU & RAM usage at ~1 Hz."""
    def __init__(self, cpu_limit: float, ram_limit: float):
        self.cpu_limit = cpu_limit
        self.ram_limit = ram_limit
        self._cpu = 0.0
        self._ram = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._sample_loop, daemon=True)
        self._thread.start()

    def _sample_loop(self):
        while not self._stop.is_set():
            cpu = psutil.cpu_percent(interval=1.0)
            ram = psutil.virtual_memory().percent
            with self._lock:
                self._cpu = cpu
                self._ram = ram

    @property
    def cpu(self) -> float:
        with self._lock:
            return self._cpu

    @property
    def ram(self) -> float:
        with self._lock:
            return self._ram

    def is_overloaded(self) -> bool:
        return self.cpu >= self.cpu_limit or self.ram >= self.ram_limit

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)

# ── Per-run worker ────────────────────────────────────────────────────────────
def run_sim(seed: int, port_base: int, exe_path: str, timeout: float = 300.0):
    """
    Launches a single headless Drone_Sim process.
    Returns (seed, internal_time_s, wall_time_s, success: bool).
    """
    start = time.perf_counter()
    try:
        result = subprocess.run(
            [exe_path, "--headless", "--seed", str(seed), "--port", str(port_base)],
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        wall = time.perf_counter() - start
        internal = -1.0
        coverage = 0.0
        for line in result.stdout.splitlines():
            if line.startswith("TIME:"):
                try:
                    internal = float(line.split("TIME:")[1].strip().rstrip("s"))
                except ValueError: pass
            if line.startswith("COVERAGE:"):
                try:
                    coverage = float(line.split("COVERAGE:")[1].strip().rstrip("%"))
                except ValueError: pass
            if line.startswith("PROGRESS:"):
                try:
                    p = float(line.split("PROGRESS:")[1].strip().rstrip("%"))
                    if p > coverage: coverage = p
                except ValueError: pass
        return seed, internal, wall, coverage, result.returncode == 0 and internal >= 0
    except subprocess.TimeoutExpired:
        wall = time.perf_counter() - start
        return seed, -1.0, wall, 0.0, False
    except Exception as e:
        wall = time.perf_counter() - start
        return seed, -1.0, wall, 0.0, False

# ── Throttle helper ───────────────────────────────────────────────────────────
def wait_for_headroom(monitor: ResourceMonitor, check_interval: float = 0.5):
    """Block until system resources drop below limits."""
    while monitor.is_overloaded():
        time.sleep(check_interval)

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Drone Sim Study Orchestrator (Sequential)")
    parser.add_argument("--runs",       type=int,   default=50,              help="Number of simulation runs")
    parser.add_argument("--port-base",  type=int,   default=10000,           help="Starting UDP port (20 ports per run)")
    parser.add_argument("--seed-start", type=int,   default=100001,          help="First seed value")
    parser.add_argument("--timeout",    type=float, default=300.0,           help="Per-run timeout (seconds)")
    parser.add_argument("--output",     type=str,   default="studies/results.csv", help="Output CSV path")
    args = parser.parse_args()

    # Find the Drone_Sim binary
    exe = "target/release/Drone_Sim.exe" if sys.platform == "win32" else "target/release/Drone_Sim"
    if not os.path.exists(exe):
        print(f"{RED}ERROR:{RESET} Binary not found at '{exe}'. Run `cargo build --release` first.")
        sys.exit(1)

    # Resource monitor just for display — no throttling needed in sequential mode
    monitor = ResourceMonitor(cpu_limit=100.0, ram_limit=100.0)

    seeds = [args.seed_start + i for i in range(args.runs)]
    ports = [args.port_base + i * 20 for i in range(args.runs)]

    print(f"\n{BOLD}╔══════════════════════════════════════════════════╗{RESET}")
    print(f"{BOLD}║   Drone Sim Study Orchestrator  [SEQUENTIAL]     ║{RESET}")
    print(f"{BOLD}╠══════════════════════════════════════════════════╣{RESET}")
    print(f"{BOLD}║{RESET}  Runs:    {CYAN}{args.runs}{RESET} (one at a time)")
    print(f"{BOLD}║{RESET}  Output:  {CYAN}{args.output}{RESET}")
    print(f"{BOLD}╚══════════════════════════════════════════════════╝\n{RESET}")

    os.makedirs(os.path.dirname(args.output) if os.path.dirname(args.output) else ".", exist_ok=True)

    results  = []
    completed = 0
    failed    = 0
    wall_start = time.perf_counter()

    # ── Sequential loop — one sim at a time ──────────────────────────────────
    for seed, port in zip(seeds, ports):
        r_seed, internal, wall, coverage, success = run_sim(seed, port, exe, args.timeout)

        results.append({
            "seed":            r_seed,
            "internal_time_s": f"{internal:.4f}",
            "wall_time_s":     f"{wall:.4f}",
            "coverage_pct":    f"{coverage:.2f}",
            "success":         "1" if success else "0",
            "cpu_pct":         f"{monitor.cpu:.1f}",
            "ram_pct":         f"{monitor.ram:.1f}",
        })
        completed += 1
        if not success:
            failed += 1

        status   = f"{GREEN}✅{RESET}" if success else f"{RED}❌{RESET}"
        time_str = f"{internal:.2f}s" if success else "FAILED"
        print(
            f"  {status} [{completed:>3}/{args.runs}] Seed {r_seed:>7} | "
            f"sim={time_str:<8} wall={wall:.1f}s | "
            f"CPU {bar(monitor.cpu)} {monitor.cpu:4.1f}%  "
            f"RAM {bar(monitor.ram)} {monitor.ram:4.1f}%"
        )

    monitor.stop()
    total_wall = time.perf_counter() - wall_start

    # Write CSV
    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["seed","internal_time_s","wall_time_s","coverage_pct","success","cpu_pct","ram_pct"])
        writer.writeheader()
        writer.writerows(results)

    successful = [float(r["internal_time_s"]) for r in results if r["success"] == "1"]
    mean   = sum(successful) / len(successful) if successful else 0
    srt    = sorted(successful)
    median = srt[len(srt)//2] if srt else 0
    mn, mx = (srt[0], srt[-1]) if srt else (0, 0)

    print(f"\n{BOLD}╔══════════════════════════════════════════════════╗{RESET}")
    print(f"{BOLD}║                  STUDY RESULTS                    ║{RESET}")
    print(f"{BOLD}╠══════════════════════════════════════════════════╣{RESET}")
    print(f"{BOLD}║{RESET}  Runs:        {GREEN}{completed - failed}{RESET}/{args.runs} succeeded")
    print(f"{BOLD}║{RESET}  Mean Time:   {CYAN}{mean:.2f}s{RESET}")
    print(f"{BOLD}║{RESET}  Median:      {CYAN}{median:.2f}s{RESET}")
    print(f"{BOLD}║{RESET}  Min / Max:   {CYAN}{mn:.2f}s / {mx:.2f}s{RESET}")
    print(f"{BOLD}║{RESET}  Total Wall:  {CYAN}{total_wall:.1f}s{RESET}")
    print(f"{BOLD}╚══════════════════════════════════════════════════╝{RESET}")
    print(f"\n  ✅  Results saved to {CYAN}{args.output}{RESET}")
    print(f"  📊  Run {CYAN}python studies/generate_chart.py{RESET} to plot.\n")

if __name__ == "__main__":
    main()
