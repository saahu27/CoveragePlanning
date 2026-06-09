# Adaptive GPR Coverage (TurtleBot3)

ROS 2 Jazzy stack for simulated GPR scanning: user-defined scan region → boustrophedon plan → online LiDAR mapping → segment invalidation → schedule replan → Nav2 execution.

**Documentation:** [`docs/README.md`](docs/README.md) · **Technical note:** [`docs/TECHNICAL_REFERENCE.md`](docs/TECHNICAL_REFERENCE.md)  
Build PDFs (dev container): `./docs/verify-docs.sh`

## Demo video

Recorded Gazebo + RViz run (5× speed): [`media/demo_5x.mp4`](media/demo_5x.mp4)

> If the video does not download when you clone, install [Git LFS](https://git-lfs.com/) and run `git lfs pull`.

---

## Run the demo

**You need:** Linux, Docker, and a local X11 display (Gazebo and RViz windows).

**What you should see:** initial zig-zag coverage path → obstacles appear in the local grid → blocked segments dropped → updated path → robot continues scanning. Coverage reports are written to `results/`.

### Option 1 — VS Code / Dev Container

1. Install [Docker](https://docs.docker.com/engine/install/) and the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
2. Clone this repo and open the folder in VS Code.
3. **Dev Containers: Reopen in Container**  
   First open builds the image and runs `colcon build` (can take 10–20 minutes).
4. In the container terminal:

```bash
source /overlay_ws/install/setup.bash
ros2 launch gpr_bringup gpr_coverage.launch.py
```

Polygon world (optional):

```bash
ros2 launch gpr_bringup gpr_coverage.launch.py scenario:=polygon
```

### Option 2 — Docker Compose (no IDE)

From the repo root on the host:

```bash
xhost +local:docker
docker compose build dev
docker compose run --rm dev bash
```

Inside the container:

```bash
cd /overlay_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch gpr_bringup gpr_coverage.launch.py
```

---

## Configuration

Mission parameters: [`gpr_bringup/config/gpr_coverage.yaml`](gpr_bringup/config/gpr_coverage.yaml)  
Nav2 parameters: [`gpr_bringup/config/nav2_gpr_params.yaml`](gpr_bringup/config/nav2_gpr_params.yaml)

After changing the Dockerfile: `docker compose build dev`

---

## Packages

`gpr_common` · `gpr_msgs` · `gpr_perception` · `gpr_planning` · `gpr_control` · `gpr_mission` · `gpr_metrics` · `gpr_bringup`

---

## License

MIT — Copyright (c) 2024-2026 Sahruday Patti
