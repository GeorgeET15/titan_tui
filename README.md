# TITAN CONTROL SYSTEM v0.1.0 🦀

The **TITAN Control System** is a high-performance, Text-based User Interface (TUI) for the TITAN robot dashboard, built using **Rust** and the `ratatui` crate.

## ✨ Features
- **Professional Startup Screen**: Performs a 3-second diagnostic check of Battery, USB components, and ROS context.
- **Live ROS 2 Telemetry**: Subscribes to `/odom` to display real-time coordinates (X, Y) and Orientation (Yaw).
- **Intelligent Operations Menu**: Grouped commands for [Robot] operations, [Sys] maintenance, and [Laptop] instructions.
- **Dynamic Color-Coding**: Visual alerts for hardware status (Green/Yellow/Red) and human-readable power diagnostics.
- **Smart Tmux Integration**: Automatically splits a vertical pane for long-running ROS processes.

## 🛠️ Installation
Ensure you have the Rust toolchain and `libclang-dev` installed:

```bash
cd ~/titan_tui
cargo build
```

## 🚀 Usage
Launch the dashboard from a `tmux` session for best results:
```bash
tmux
cargo run
```

## 🎮 Interface Controls
- **Arrows**: Navigate Menu
- **Enter**: Execute Selected Command
- **Q**: Quit
- **C**: Clear Logs
