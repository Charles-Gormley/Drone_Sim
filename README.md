# Drone Mesh Networking Simulator

An autonomous multi-drone exploration simulator implementing a **peer-to-peer mesh network** using the **Noise Protocol Framework** (XX handshake, AES-GCM-256). Each drone independently explores an unknown environment, shares encrypted map data with peers, and coordinates pathing to avoid redundant coverage.

---

## Quick Start

### Prerequisites

- [Rust](https://rustup.rs/) (stable, 1.70+)
- Windows, macOS, or Linux

### Run

```bash
git clone <repo-url>
cd Drone_Sim
cargo run
```

> First build will take ~60 seconds to compile cryptographic dependencies (`snow`, `curve25519-dalek`). Subsequent builds are incremental and fast.

---

## Controls

| Key | Action |
|-----|--------|
| `C` | Cycle camera between drones |
| `Z` | Toggle god-mode zoom (see full map) |
| `O` | Toggle map overlay (raw environment vs. explored quadtree) |
| `V` | Toggle path/frontier visualization |

---

## Configuration

All tunable constants are at the top of their respective files:

| Constant | File | Default | Description |
|----------|------|---------|-------------|
| `DRONE_COUNT` | `main.rs` | `3` | Number of drones to spawn |
| `BASE_PORT` | `drone.rs` | `8000` | UDP base port (drone N uses `BASE_PORT + N`) |
| `MAX_DRONES` | `drone.rs` | `10` | Max swarm size / port scan range |
| `DEAD_PEER_TICKS` | `drone.rs` | `625` | Ticks before a silent peer is culled (~10s) |
| `GRID_RES` | `autonomy.rs` | `25.0` | Pathfinding grid resolution in world pixels |
| `MOVE_SPEED` | `drone.rs` | `500.0` | Maximum drone velocity |

---

## Architecture

See [`docs/architecture.md`](docs/architecture.md) for full system diagrams.

---

## Project Structure

```
Drone_Sim/
├── src/
│   ├── main.rs          # Entry point, render loop, UI controls
│   ├── drone.rs         # Drone actor: physics, UDP networking, peer table
│   ├── network.rs       # Noise Protocol session table, wire types
│   ├── autonomy.rs      # A* pathfinding, frontier selection, swarm coordination
│   ├── quadtree.rs      # Spatial map: scan ingestion, diff/merge for gossip
│   └── environment.rs   # Static obstacle map generation and rendering
├── docs/
│   └── architecture.md  # System and data flow diagrams
└── .agent/
    └── tracker.md       # Development roadmap
```
