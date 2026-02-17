# TRIDENT TITAN v0.1.0 🔱🚀

**TRIDENT** (**Tactical Remote Interface for Detailed Exploration and Navigation TITAN**) is a high-performance, premium cockpit for the TITAN robot suite. Built with **Rust** and **ROS 2**, it provides a military-grade tactical interface for real-time mission control and autonomous navigation management.

## ✨ Features
- **Dynamic Teleop Configuration**: Tune linear speed and angular turn profiles on the fly via a sleek modal dialog before launching control nodes.
- **Interactive Map Selection**: Automatically scans the robot's maps directory and allows selecting specific `.yaml` maps for navigation.
- **Tactical Dashboard**: Re-engineered UI with neutral tones for high visibility, reduced eye strain, and color-coded survival telemetry.
- **Live ROS 2 Telemetry**: Subscribes directly to `/odom` for real-time tracking of Position (X, Y) and Orientation (Theta).
- **Proactive Diagnostics**: Automated hardware and driver checks displayed during a diagnostic splash sequence.
- **Global Deployment**: Effortless setup via an automated installer, exposing the `trident` command system-wide.

## 🛠️ Installation & Setup
Ensure you have the Rust toolchain and ROS 2 Jazzy (or compatible) installed.

### 1. One-Command Global Install
The easiest way to set up TRIDENT on any device (Robot or Laptop):
```bash
chmod +x install.sh
./install.sh
```

### 2. Manual Installation
Alternatively, use Cargo directly:
```bash
cargo install --path . --force
```

### 3. Launch
TRIDENT is optimized for `tmux` environments but runs in any modern terminal:
```bash
# From any directory:
trident
```

## 🎮 Interface Controls
- **Arrows / Tab**: Navigate Menus & Input Fields
- **Enter**: Execute / Confirm / Next Field
- **Q**: Tactical Exit (Quit)
- **C**: Clear System Logs
- **Esc**: Cancel Modal / Back to Main

## 🏗️ Technical Stack
- **Core**: Rust (Safety & Performance)
- **UI**: `ratatui` (Custom Tactical Frame)
- **ROS 2**: `r2r` (Native Odometry Integration)
- **Shell**: Bash (Automated Deployment)
