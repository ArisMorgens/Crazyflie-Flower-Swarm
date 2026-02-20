# Crazyflie Flower Swarm

Generates 3D flower trajectories for a Crazyflie drone swarm.
`flower.py` produces the trajectory JSON files consumed by the Rust swarm script.

## Python — generate trajectories

### With uv (recommended)

```bash
uv sync
uv run flower.py
```

### With pip

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install .
python3 flower.py
```

No system packages required — all dependencies install via pip/uv.

Running `flower.py` will:
- Display an interactive 3D plot of the flowers
- Write `flower_trajectories/stem{n}.json` and `flower_trajectories/petals{n}.json`

## Rust — fly the swarm

Requires [Rust](https://rustup.rs) and a Crazyradio dongle.

1. Set the drone URIs in `examples/flower_swarm_compressed.rs` (`URIS` constant)
2. Generate the trajectories with `flower.py` (see above)
3. Run:

```bash
cargo run --example flower_swarm_compressed
```
