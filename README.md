# TRIDENT TUI v0.1.0 🔱

**TRIDENT** (**Tactical Remote Interface for Detailed Exploration and Navigation TITAN**) is a high-performance, premium dashboard for the TITAN robot suite. Built with **Rust** and **ROS 2**, it provides a military-grade tactical interface for real-time mission control.

## ✨ Features
- **High-Impact Tactical Interface**: A redesigned "Wow-factor" banner and main dashboard with thick borders and vibrant highlights.
- **Titan Control Center**: Centralized management for [Robot] bringup, [Sys] maintenance, and [Laptop] operator tasks.
- **Live ROS 2 Telemetry**: Subscribes directly to `/odom` for real-time tracking of Position (X, Y) and Orientation (Theta).
- **Global Terminal Command**: Launch the full system from anywhere by typing `trident`.
- **Intelligent Logging**: Real-time status stream with color-coded severity levels (Critical/Stable/Capped).

## 🛠️ Installation & Setup
Ensure you have the Rust toolchain and `libclang-dev` installed for ROS 2 bindings.

### 1. Build & Install Globally
Run this from the project root to enable the `trident` command everywhere:
```bash
cargo install --path .
```

### 2. Launch
The TUI is optimized for `tmux` environments but runs in any modern terminal:
```bash
# From any directory:
trident
```

## 🎮 Interface Controls
- **Arrows**: Navigate Menu
- **Enter**: Execute Command
- **Q**: Tactical Exit (Quit)
- **C**: Clear System Logs

## 🏗️ Technical Stack
- **Core**: Rust (Safety & Performance)
- **UI**: `ratatui` (Custom Tactical Frame)
- **ROS 2**: `r2r` (Native Odometry Integration)
