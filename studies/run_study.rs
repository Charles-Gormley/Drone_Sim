/// run_study.rs — Swarm Exploration Study Orchestrator (Rust)
///
/// Runs many headless Drone_Sim instances in parallel, dynamically throttling
/// concurrency to keep the host CPU and RAM below configurable limits.
///
/// CPU/RAM measurements are sampled on a background thread and written into
/// the output CSV alongside timing data.
///
/// Usage (from project root):
///     cargo run --release --bin run_study
///
/// Or with overrides:
///     cargo run --release --bin run_study -- --runs 200 --concurrency 8 --cpu-limit 55

use std::process::Command;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;
use tokio::sync::Semaphore;
use tokio::task;
use sysinfo::System;

// ── Configuration constants ────────────────────────────────────────────────
const DEFAULT_RUN_COUNT:   u32   = 1000;
const DEFAULT_CONCURRENCY: usize = 8;       // conservative default for safety
const DEFAULT_CPU_LIMIT:   f32   = 60.0;   // throttle if CPU% >= this
const DEFAULT_RAM_LIMIT:   f32   = 60.0;   // throttle if RAM% >= this
const PORTS_PER_INSTANCE:  u32   = 20;
const PORT_BASE:           u32   = 10000;
const THROTTLE_SLEEP_MS:   u64   = 500;    // how long to wait before retrying when overloaded

// ── Resource snapshot shared across threads ────────────────────────────────
#[derive(Clone, Default)]
struct ResourceSnapshot {
    cpu_pct: f32,
    ram_pct: f32,
}

type SharedResources = Arc<Mutex<ResourceSnapshot>>;

// ── Background sampler ─────────────────────────────────────────────────────
fn spawn_resource_monitor(shared: SharedResources) {
    std::thread::spawn(move || {
        let mut sys = System::new_all();
        loop {
            sys.refresh_cpu_all();
            sys.refresh_memory();

            let cpu_pct = sys.global_cpu_usage();
            let total_mem = sys.total_memory();
            let used_mem  = sys.used_memory();
            let ram_pct = if total_mem > 0 {
                used_mem as f32 / total_mem as f32 * 100.0
            } else {
                0.0
            };

            {
                let mut snap = shared.lock().unwrap();
                snap.cpu_pct = cpu_pct;
                snap.ram_pct = ram_pct;
            }

            std::thread::sleep(std::time::Duration::from_millis(800));
        }
    });
}

// ── CLI argument parsing (minimal, no extra deps) ─────────────────────────
struct Config {
    run_count:   u32,
    concurrency: usize,
    cpu_limit:   f32,
    ram_limit:   f32,
}

fn parse_args() -> Config {
    let args: Vec<String> = std::env::args().collect();
    let get = |flag: &str| -> Option<String> {
        args.iter().position(|a| a == flag)
            .and_then(|i| args.get(i + 1).cloned())
    };

    Config {
        run_count:   get("--runs").and_then(|v| v.parse().ok()).unwrap_or(DEFAULT_RUN_COUNT),
        concurrency: get("--concurrency").and_then(|v| v.parse().ok()).unwrap_or(DEFAULT_CONCURRENCY),
        cpu_limit:   get("--cpu-limit").and_then(|v| v.parse().ok()).unwrap_or(DEFAULT_CPU_LIMIT),
        ram_limit:   get("--ram-limit").and_then(|v| v.parse().ok()).unwrap_or(DEFAULT_RAM_LIMIT),
    }
}

// ── Result row ────────────────────────────────────────────────────────────
struct RunResult {
    seed:          u32,
    internal_time: f32,  // -1 = failure
    wall_time:     f32,
    coverage:      f32,  // percentage
    cpu_at_end:    f32,
    ram_at_end:    f32,
}

// ── Main ──────────────────────────────────────────────────────────────────
#[tokio::main]
async fn main() {
    let cfg = parse_args();

    let exe_path = if cfg!(windows) {
        "target/release/Drone_Sim.exe"
    } else {
        "target/release/Drone_Sim"
    };

    if !std::path::Path::new(exe_path).exists() {
        eprintln!(
            "ERROR: Drone_Sim binary not found at '{}'. Run `cargo build --release` first.",
            exe_path
        );
        std::process::exit(1);
    }

    // Start background resource monitor
    let resources: SharedResources = Arc::new(Mutex::new(ResourceSnapshot::default()));
    spawn_resource_monitor(resources.clone());

    // Brief warm-up so the first CPU sample is non-zero
    tokio::time::sleep(std::time::Duration::from_millis(1200)).await;

    println!("╔══════════════════════════════════════════════════╗");
    println!("║      Swarm Exploration Study — Rust Orchestrator  ║");
    println!("╠══════════════════════════════════════════════════╣");
    println!("║  Runs:        {:<34}║", cfg.run_count);
    println!("║  Concurrency: {:<34}║", cfg.concurrency);
    println!("║  CPU limit:   {:<34}║", format!("{:.0}%", cfg.cpu_limit));
    println!("║  RAM limit:   {:<34}║", format!("{:.0}%", cfg.ram_limit));
    println!("╚══════════════════════════════════════════════════╝\n");

    let wall_start = Instant::now();
    let semaphore  = Arc::new(Semaphore::new(cfg.concurrency));
    let completed  = Arc::new(AtomicUsize::new(0));

    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<RunResult>();

    let mut tasks = Vec::with_capacity(cfg.run_count as usize);

    for i in 1..=cfg.run_count {
        // ── throttle: wait until system has headroom ──────────────────────
        loop {
            let (cpu, ram) = {
                let snap = resources.lock().unwrap();
                (snap.cpu_pct, snap.ram_pct)
            };
            if cpu < cfg.cpu_limit && ram < cfg.ram_limit {
                break;
            }
            // Print a throttle warning then wait
            println!(
                "  ⚠  THROTTLE  CPU={:.1}% RAM={:.1}% — waiting for headroom...",
                cpu, ram
            );
            tokio::time::sleep(std::time::Duration::from_millis(THROTTLE_SLEEP_MS)).await;
        }

        let permit       = semaphore.clone().acquire_owned().await.unwrap();
        let exe          = exe_path.to_string();
        let completed_c  = completed.clone();
        let resources_c  = resources.clone();
        let tx_c         = tx.clone();
        let seed         = 100_000 + i;
        let base_port    = PORT_BASE + (i * PORTS_PER_INSTANCE);
        let run_count    = cfg.run_count;

        tasks.push(task::spawn_blocking(move || {
            let run_start = Instant::now();

            let output = Command::new(&exe)
                .args(&[
                    "--headless",
                    "--seed",  &seed.to_string(),
                    "--port",  &base_port.to_string(),
                ])
                .output();

            let wall_time = run_start.elapsed().as_secs_f32();

            let mut internal_time = -1.0f32;
            let mut coverage      = 0.0f32;
            if let Ok(out) = output {
                let stdout = String::from_utf8_lossy(&out.stdout);
                for line in stdout.lines() {
                    if let Some(rest) = line.strip_prefix("TIME: ") {
                        if let Ok(t) = rest.trim_end_matches('s').trim().parse::<f32>() {
                            internal_time = t;
                        }
                    }
                    if let Some(rest) = line.strip_prefix("COVERAGE: ") {
                        if let Ok(c) = rest.trim_end_matches('%').trim().parse::<f32>() {
                            coverage = c;
                        }
                    }
                    if let Some(rest) = line.strip_prefix("PROGRESS: ") {
                        if let Ok(c) = rest.trim_end_matches('%').trim().parse::<f32>() {
                            if c > coverage { coverage = c; }
                        }
                    }
                }
            }

            let (cpu_snap, ram_snap) = {
                let snap = resources_c.lock().unwrap();
                (snap.cpu_pct, snap.ram_pct)
            };

            let count = completed_c.fetch_add(1, Ordering::Relaxed) + 1;
            if count % 50 == 0 || count == run_count as usize {
                println!(
                    "  ▶  [{}/{}]  CPU={:.1}%  RAM={:.1}%",
                    count, run_count, cpu_snap, ram_snap
                );
            }

            tx_c.send(RunResult {
                seed,
                internal_time,
                wall_time,
                coverage,
                cpu_at_end: cpu_snap,
                ram_at_end: ram_snap,
            }).unwrap();

            drop(permit);
        }));
    }

    drop(tx); // signal end of stream

    let mut results: Vec<RunResult> = Vec::with_capacity(cfg.run_count as usize);
    while let Some(row) = rx.recv().await {
        results.push(row);
    }
    for t in tasks { let _ = t.await; }

    // Sort by seed
    results.sort_by_key(|r| r.seed);

    // ── Write CSV ─────────────────────────────────────────────────────────
    let csv_path = "studies/results.csv";
    let mut csv = String::from("seed,internal_time_s,wall_time_s,coverage_pct,success,cpu_pct,ram_pct\n");
    for r in &results {
        let success = if r.internal_time >= 0.0 { 1 } else { 0 };
        csv.push_str(&format!(
            "{},{:.4},{:.4},{:.2},{},{:.1},{:.1}\n",
            r.seed, r.internal_time, r.wall_time, r.coverage, success, r.cpu_at_end, r.ram_at_end
        ));
    }
    std::fs::write(csv_path, csv).expect("Failed to write studies/results.csv");

    // ── Summary statistics ────────────────────────────────────────────────
    let success_times: Vec<f32> = results.iter()
        .filter(|r| r.internal_time >= 0.0)
        .map(|r| r.internal_time)
        .collect();

    let success_count = success_times.len();
    let mean   = if success_count > 0 { success_times.iter().sum::<f32>() / success_count as f32 } else { 0.0 };
    let mut st = success_times.clone();
    st.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = st.get(st.len() / 2).copied().unwrap_or(0.0);
    let min    = st.first().copied().unwrap_or(0.0);
    let max    = st.last().copied().unwrap_or(0.0);

    let cpu_readings: Vec<f32> = results.iter().map(|r| r.cpu_at_end).collect();
    let ram_readings: Vec<f32> = results.iter().map(|r| r.ram_at_end).collect();
    let peak_cpu = cpu_readings.iter().cloned().fold(0.0_f32, f32::max);
    let peak_ram = ram_readings.iter().cloned().fold(0.0_f32, f32::max);
    let avg_cpu  = cpu_readings.iter().sum::<f32>() / cpu_readings.len() as f32;
    let avg_ram  = ram_readings.iter().sum::<f32>() / ram_readings.len() as f32;

    let total_wall = wall_start.elapsed();

    println!("\n╔══════════════════════════════════════════════════╗");
    println!("║                  STUDY RESULTS                    ║");
    println!("╠══════════════════════════════════════════════════╣");
    println!("║  Runs:        {:<34}║", format!("{}/{}", success_count, cfg.run_count));
    println!("║  Mean Time:   {:<34}║", format!("{:.2}s", mean));
    println!("║  Median:      {:<34}║", format!("{:.2}s", median));
    println!("║  Min / Max:   {:<34}║", format!("{:.2}s / {:.2}s", min, max));
    println!("║  Total Wall:  {:<34}║", format!("{:.1}s", total_wall.as_secs_f32()));
    println!("╠══════════════════════════════════════════════════╣");
    println!("║  Peak CPU:    {:<34}║", format!("{:.1}%  (limit: {:.0}%)", peak_cpu, cfg.cpu_limit));
    println!("║  Peak RAM:    {:<34}║", format!("{:.1}%  (limit: {:.0}%)", peak_ram, cfg.ram_limit));
    println!("║  Avg CPU:     {:<34}║", format!("{:.1}%", avg_cpu));
    println!("║  Avg RAM:     {:<34}║", format!("{:.1}%", avg_ram));
    println!("╚══════════════════════════════════════════════════╝");
    println!("\n  ✅  Results saved to {}", csv_path);
    println!("  📊  Run `python studies/generate_chart.py` to plot.");
}
