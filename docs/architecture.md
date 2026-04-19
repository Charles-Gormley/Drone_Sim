# System Architecture

## Overview

The simulator is structured as a **multi-actor system**. The main thread owns the render loop (macroquad / OpenGL). A background OS thread owns a Tokio async runtime, inside which each drone runs as an independent `tokio::task`. Communication between the render thread and drone tasks is strictly one-way: drones push state updates through an **unbounded MPSC channel**.

---

## 1. Process & Thread Model

```mermaid
graph TD
    subgraph "OS Thread: Main (macroquad)"
        RL[Render Loop\n60fps]
        UI[UI / Camera Controls]
        MM[Master Map\nQuadtree]
    end

    subgraph "OS Thread: Tokio Runtime"
        subgraph "tokio::task — Drone 1"
            D1[Physics\nUpdate]
            D1N[UDP\nSocket :8001]
            D1A[A* Autonomy\n250ms]
        end
        subgraph "tokio::task — Drone 2"
            D2[Physics\nUpdate]
            D2N[UDP\nSocket :8002]
            D2A[A* Autonomy\n250ms]
        end
        subgraph "tokio::task — Drone N"
            DN[Physics\nUpdate]
            DNN[UDP\nSocket :800N]
            DNA[A* Autonomy\n250ms]
        end
    end

    D1 -- "RenderState + MapDiff\n(mpsc channel)" --> RL
    D2 -- "RenderState + MapDiff\n(mpsc channel)" --> RL
    DN -- "RenderState + MapDiff\n(mpsc channel)" --> RL
    RL --> MM
    RL --> UI
```

> **Key design decision:** The render thread and drone tasks share **zero mutable state**. The quadtree is duplicated: each drone owns its own, and the render thread owns a `master_map` that merges diffs as they arrive. This avoids any `Arc<Mutex<T>>` and its associated lock contention.

---

## 2. Drone Actor Internals

Each drone task runs a `tokio::select!` loop with four concurrent arms:

```mermaid
graph LR
    subgraph "Drone Task — tokio::select!"
        T1["⏱ ticker\n16ms\nPhysics + Scan"]
        T2["⏱ heartbeat_ticker\n2s\nBroadcast UDP Heartbeat"]
        T3["⏱ map_gossip_ticker\n5s\nEncrypted MAP_DIFF"]
        T4["📡 socket.recv_from\nIncoming UDP"]
    end

    T1 --> PHYS[update_physics\nSeek + Collision]
    T1 --> AUT[autonomy.update\nA* Repath]
    T1 --> SCAN[quadtree.insert_scan]
    T1 --> CULL[peer_table.retain\nDead Peer Culling]

    T2 --> HB[Plaintext Heartbeat\nto all ports in range]
    T2 --> HS[Noise XX Handshake\ninitiation if no session]

    T3 --> ENC[Encrypt with\nNoiseSessionTable]
    T3 --> SEND[send_to peers\nwith active session]

    T4 --> PARSE{Envelope tag?}
    PARSE -- "0x01 HANDSHAKE" --> HSP[process_handshake_msg]
    PARSE -- "0x02 TRANSPORT" --> DEC[Decrypt + process_message]
    PARSE -- "Plaintext" --> PM[process_message]
```

---

## 3. Network Protocol Stack

```mermaid
sequenceDiagram
    participant D1 as Drone 1 (id=1, lower)
    participant D2 as Drone 2 (id=2, higher)

    Note over D1,D2: Phase 1 — Discovery (Plaintext)
    D2->>D1: UDP Heartbeat (MsgType::Heartbeat, plaintext)
    D1->>D2: UDP Heartbeat (MsgType::Heartbeat, plaintext)

    Note over D1,D2: Phase 2 — Noise XX Handshake (3 messages)
    Note over D2: Higher ID always initiates
    D2->>D1: Envelope{tag=HANDSHAKE, msg1}
    D1->>D2: Envelope{tag=HANDSHAKE, msg2}
    D2->>D1: Envelope{tag=HANDSHAKE, msg3}
    Note over D1,D2: Both sides now have TransportState ✅

    Note over D1,D2: Phase 3 — Encrypted Data Exchange
    D2->>D1: Envelope{tag=TRANSPORT, AES-GCM ciphertext}
    Note over D1: decrypt → MapDiff → quadtree.merge()
    D1->>D2: Envelope{tag=TRANSPORT, AES-GCM ciphertext}
    Note over D2: decrypt → MapDiff → quadtree.merge()
```

**Role assignment rule:** The drone with the *higher* numeric ID always acts as the Noise **initiator**. The lower-ID drone waits and acts as **responder**. This is enforced with a single filter:
```rust
.filter(|&pid| pid > self.id)  // only initiate to higher-ID peers
```
This prevents both drones simultaneously trying to be initiator, which would corrupt the handshake state machine.

---

## 4. Swarm Coordination (Frontier Selection)

```mermaid
flowchart TD
    START([Every 250ms per drone]) --> GRID[Rasterize Quadtree\nto CELL_UNKNOWN / EXPLORED / OBSTACLE grid]
    GRID --> FF[find_frontiers\nEdge cells between EXPLORED and UNKNOWN]
    FF --> SCORE[Score each frontier]

    SCORE --> D[Base score =\ndistance² from self]
    D --> PP[Peer Position Penalty\n−= if peer within 500px]
    PP --> PA[Peer Path Penalty\n−= if peer waypoint within 300px]
    PA --> PICK[Pick lowest score\nskip banned frontiers]

    PICK --> ASTAR[A* Pathfind\nthrough EXPLORED cells only]
    ASTAR --> |Path found| FOLLOW[Follow path\n15px waypoint radius]
    ASTAR --> |Unreachable| BAN[Ban frontier\nrecalculate next tick]
```

**Key insight:** Drones do not *negotiate* paths. Each drone independently applies a penalty to frontiers that peer drones are already heading toward, causing the swarm to naturally partition the map without a central coordinator.

---

## 5. Map Data Flow

```mermaid
graph LR
    SCAN["insert_scan()\nRadial raycast\nat scan_radius=300px"] --> QT[Drone's Local\nQuadtree]
    QT --> DIFF["diff_since(last_tick)\nIncremental delta"]
    DIFF --> SER[bincode::serialize\nMapDiff]
    SER --> ENC[Noise encrypt]
    ENC --> UDP[UDP send_to peers]

    UDP2[UDP recv] --> DEC2[Noise decrypt]
    DEC2 --> DE[bincode::deserialize\nMapDiff]
    DE --> MERGE["quadtree.merge()\nPeer's map integrated"]

    QT2["Each drone's\nQuadtree"] --> CH["mpsc channel\nMapDiff"] --> MM["Master Map\n(render thread)"]
```

---

## 6. Data Structures

| Structure | Owner | Purpose |
|-----------|-------|---------|
| `Quadtree` | Each `Drone` (1 copy) + `main.rs` (master) | Spatial map of explored/obstacle cells |
| `HashMap<u32, Peer>` | Each `Drone` | Live peer routing table; culled after `DEAD_PEER_TICKS` |
| `NoiseSessionTable` | Each `Drone` | Tracks handshake state and completed `TransportState` per peer |
| `AutonomyState` | Each `Drone` | A* path, target frontier, banned frontier list, repathing timers |
| `RenderState` | `main.rs` (`HashMap<u32, RenderState>`) | Latest snapshot of each drone for rendering |

---

## Phase Roadmap

| Phase | Status | Description |
|-------|--------|-------------|
| 1 | ✅ Done | Single drone, physics, quadtree mapping |
| 2 | ✅ Done | A* pathfinding, frontier exploration |
| 3 | ✅ Done | Multi-drone, async tokio actor model |
| 4 | ✅ Done | UDP gossip, Noise Protocol encryption, swarm coordination |
| 5 | 🔲 Next | Resilience: drone kill, RF jamming simulation |
| 6 | 🔲 Planned | Interactive demo: live spawn/kill, packet loss slider |
| 7 | 🔲 Optional | Perlin noise terrain, 3D upgrade via Bevy |
