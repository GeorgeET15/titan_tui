use ratatui::{
    backend::CrosstermBackend,
    widgets::{Block, Borders, Paragraph, List, ListItem, Gauge, BorderType},
    layout::{Layout, Constraint, Direction, Alignment, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    Terminal,
};
use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use anyhow::Result;
use std::{io, time::Duration, process::{Command, Child, Stdio}, fs};
use futures::StreamExt;
use r2r::nav_msgs::msg::Odometry;
use r2r::geometry_msgs::msg::{PoseStamped, PoseWithCovarianceStamped, Quaternion};
use r2r::builtin_interfaces::msg::Time;
use tokio::sync::{watch, mpsc};
use serde::{Serialize, Deserialize};
use r2r::std_msgs::msg::Empty as EmptyMsg;

#[derive(Clone, Default, Serialize, Deserialize, Debug)]
struct Waypoint {
    name: String,
    x: f64,
    y: f64,
    theta: f64,
}

#[derive(Clone, Default)]
struct Telemetry {
    x: f64,
    y: f64,
    theta: f64,
    stamp: Time,
}

#[derive(Clone, PartialEq, Eq)]
enum MenuItem {
    // Robot Local
    Bringup,
    LocalTeleop,
    Mapping,
    Cartographer,
    Navigation,
    Waypoints,
    SaveMap,
    // System
    CheckUSB,
    Battery,
    Rebuild,
    ConnectWiFi,
    KillAll,
    // Laptop Remote (Instructions)
    RemoteTeleop,
    RemoteRViz,
}
impl MenuItem {
    fn to_string(&self) -> String {
        match self {
            MenuItem::Bringup => "[Robot] START BRINGUP".to_string(),
            MenuItem::LocalTeleop => "[Robot] LOCAL TELEOP (TMUX)".to_string(),
            MenuItem::Mapping => "[Robot] START MAPPING (SLAM TOOLBOX)".to_string(),
            MenuItem::Cartographer => "[Robot] START MAPPING (CARTOGRAPHER)".to_string(),
            MenuItem::Navigation => "[Robot] START NAV2".to_string(),
            MenuItem::Waypoints => "[Robot] WAYPOINTS / NAV ".to_string(),
            MenuItem::SaveMap => "[Robot] SAVE MAP".to_string(),
            MenuItem::CheckUSB => "[Sys] CHECK USB/SERIAL".to_string(),
            MenuItem::Battery => "[Sys] POWER/BATTERY".to_string(),
            MenuItem::Rebuild => "[Sys] REBUILD WORKSPACE".to_string(),
            MenuItem::ConnectWiFi => "[Sys] CONNECT WIFI (USB)".to_string(),
            MenuItem::KillAll => "[Sys] KILL ALL ROS2 PROC".to_string(),
            MenuItem::RemoteTeleop => "[Laptop] REMOTE TELEOP".to_string(),
            MenuItem::RemoteRViz => "[Laptop] REMOTE RVIZ".to_string(),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum DeviceType {
    Titan,
    Laptop,
    Unselected,
}

impl DeviceType {
    fn to_string(&self) -> &str {
        match self {
            DeviceType::Titan => "TITAN (Robot)",
            DeviceType::Laptop => "LAPTOP (Operator)",
            DeviceType::Unselected => "NOT SELECTED",
        }
    }
}

#[derive(PartialEq)]
enum Screen {
    Banner,
    Splash,
    DeviceSelect,
    MapNameInput,
    TeleopConfig,
    MapSelect,
    RVizConfigSelect,
    WifiScan,
    WifiPasswordInput,
    WaypointList,
    WaypointNameInput,
    Main,
}

struct App {
    logs: Vec<String>,
    all_menu_items: Vec<MenuItem>,
    active_menu_index: usize,
    active_process: Option<Child>,
    screen: Screen,
    startup_checks: Vec<String>,
    telemetry_rx: watch::Receiver<Telemetry>,
    current_telemetry: Telemetry,
    splash_start: std::time::Instant,
    device_type: DeviceType,
    selection_index: usize,
    map_name_input: String,
    teleop_speed: String,
    teleop_turn: String,
    teleop_field_index: usize,
    pending_teleop_item: Option<MenuItem>,
    available_maps: Vec<String>,
    map_selection_index: usize,
    available_rviz_configs: Vec<String>,
    rviz_config_selection_index: usize,
    operation_status: String,
    spinner_frame: usize,
    last_tick: std::time::Instant,
    // WiFi Management
    available_ssids: Vec<String>,
    wifi_selection_index: usize,
    #[allow(dead_code)]
    selected_ssid: String,
    wifi_password_input: String,
    
    // Waypoints
    waypoints: Vec<Waypoint>,
    waypoint_selection_index: usize,
    waypoint_name_input: String,
    ros_cmd_tx: mpsc::UnboundedSender<RosCommand>,

    // UX Enhancements
    is_loading: bool,
    loading_message: String,
}

enum RosCommand {
    NavTo(Waypoint),
    SetPose(Waypoint),
    ResetOdom,
}

#[derive(Serialize, Deserialize)]
struct WaypointFile {
    waypoints: Vec<Waypoint>,
}

impl App {
    fn new(telemetry_rx: watch::Receiver<Telemetry>, ros_cmd_tx: mpsc::UnboundedSender<RosCommand>) -> App {
        let mut app = App {
            logs: vec![
                "TITAN System initialized".to_string(),
                "Ready for commands".to_string(),
            ],
            all_menu_items: vec![
                MenuItem::Bringup,
                MenuItem::LocalTeleop,
                MenuItem::Mapping,
                MenuItem::Cartographer,
                MenuItem::Navigation,
                MenuItem::Waypoints,
                MenuItem::SaveMap,
                MenuItem::CheckUSB,
                MenuItem::Battery,
                MenuItem::Rebuild,
                MenuItem::ConnectWiFi,
                MenuItem::KillAll,
                MenuItem::RemoteTeleop,
                MenuItem::RemoteRViz,
            ],
            active_menu_index: 0,
            active_process: None,
            screen: Screen::Banner,
            startup_checks: Vec::new(),
            telemetry_rx,
            current_telemetry: Telemetry::default(),
            splash_start: std::time::Instant::now(),
            device_type: DeviceType::Unselected,
            selection_index: 0,
            map_name_input: String::new(),
            teleop_speed: "0.1".to_string(),
            teleop_turn: "0.5".to_string(),
            teleop_field_index: 0,
            pending_teleop_item: None,
            available_maps: Vec::new(),
            map_selection_index: 0,
            available_rviz_configs: Vec::new(),
            rviz_config_selection_index: 0,
            operation_status: "IDLE".to_string(),
            spinner_frame: 0,
            last_tick: std::time::Instant::now(),
            available_ssids: Vec::new(),
            wifi_selection_index: 0,
            selected_ssid: String::new(),
            wifi_password_input: String::new(),
            waypoints: Vec::new(),
            waypoint_selection_index: 0,
            waypoint_name_input: String::new(),
            ros_cmd_tx,
            is_loading: false,
            loading_message: String::new(),
        };
        app.load_waypoints();
        app
    }

    fn load_waypoints(&mut self) {
        let home = std::env::var("HOME").unwrap_or_else(|_| "/home/pidev".to_string());
        let path = format!("{}/titan_ws/src/titan_bringup/config/waypoints.yaml", home);
        if let Ok(content) = fs::read_to_string(&path) {
            match serde_yaml::from_str::<WaypointFile>(&content) {
                Ok(data) => {
                    self.waypoints = data.waypoints;
                    self.logs.push(format!("Loaded {} waypoints from disk.", self.waypoints.len()));
                },
                Err(e) => {
                    self.logs.push(format!("Error parsing waypoints.yaml: {}", e));
                }
            }
        } else {
            self.logs.push("No waypoints.yaml found, starting fresh.".to_string());
        }
    }

    fn save_waypoints(&mut self) {
        let home = std::env::var("HOME").unwrap_or_else(|_| "/home/pidev".to_string());
        let path = format!("{}/titan_ws/src/titan_bringup/config/waypoints.yaml", home);
        let data = WaypointFile { waypoints: self.waypoints.clone() };
        match serde_yaml::to_string(&data) {
            Ok(content) => {
                if let Err(e) = fs::write(&path, content) {
                    self.logs.push(format!("Error saving waypoints: {}", e));
                } else {
                    self.logs.push("Waypoints successfully saved to disk.".to_string());
                }
            },
            Err(e) => {
                self.logs.push(format!("Error serializing waypoints: {}", e));
            }
        }
    }

    fn get_filtered_menu(&self) -> Vec<MenuItem> {
        self.all_menu_items.iter().filter(|item| {
            match (self.device_type, item) {
                (DeviceType::Titan, MenuItem::RemoteTeleop) | (DeviceType::Titan, MenuItem::RemoteRViz) => false,
                (DeviceType::Laptop, MenuItem::Bringup) | (DeviceType::Laptop, MenuItem::LocalTeleop) | 
                (DeviceType::Laptop, MenuItem::Mapping) | (DeviceType::Laptop, MenuItem::Cartographer) | (DeviceType::Laptop, MenuItem::Navigation) |
                (DeviceType::Laptop, MenuItem::Waypoints) |
                (DeviceType::Laptop, MenuItem::SaveMap) | (DeviceType::Laptop, MenuItem::CheckUSB) |
                (DeviceType::Laptop, MenuItem::Battery) | (DeviceType::Laptop, MenuItem::Rebuild) |
                (DeviceType::Laptop, MenuItem::ConnectWiFi) | (DeviceType::Laptop, MenuItem::KillAll) => false,
                _ => true,
            }
        }).cloned().collect()
    }

    fn run_diagnostics(&mut self) {
        self.startup_checks.push("Checking Power System...".to_string());
        self.startup_checks.push(format!("  -> {}", self.get_battery_readable()));
        
        self.startup_checks.push("Scanning Serial Bus...".to_string());
        let usb = self.get_usb_readable();
        for line in usb {
            self.startup_checks.push(format!("  -> {}", line));
        }
        
        self.startup_checks.push("ROS 2 Context... OK".to_string());
        self.startup_checks.push("Starting TITAN Control System...".to_string());
    }

    fn get_battery_readable(&self) -> String {
        let output = Command::new("vcgencmd").arg("get_throttled").output();
        if let Ok(out) = output {
            let res = String::from_utf8_lossy(&out.stdout).trim().to_string();
            if let Some(hex_str) = res.split('=').last() {
                if let Ok(val) = u32::from_str_radix(hex_str.trim_start_matches("0x"), 16) {
                    if val == 0 { return "Power Supply: STABLE (5.0V)".to_string(); }
                    let mut flags = Vec::new();
                    if val & 0x1 != 0 { flags.push("Under-voltage"); }
                    if val & 0x2 != 0 { flags.push("Freq Capped"); }
                    if val & 0x4 != 0 { flags.push("Throttling"); }
                    if val & 0x8 != 0 { flags.push("Temp Limit"); }
                    return format!("Power Warning: {}", flags.join(", "));
                }
            }
        }
        "Power Status: Unknown (Check Connection)".to_string()
    }

    fn get_usb_readable(&self) -> Vec<String> {
        let mut results = Vec::new();
        let output = Command::new("ls").arg("/dev/serial/by-id/").output();
        if let Ok(out) = output {
            let s = String::from_utf8_lossy(&out.stdout);
            let arduino = s.contains("1a86");
            let lidar = s.contains("Silicon_Labs");
            
            results.push(format!("Arduino: {}", if arduino { "CONNECTED" } else { "NOT FOUND" }));
            results.push(format!("LiDAR:   {}", if lidar { "CONNECTED" } else { "NOT FOUND" }));
        } else {
            results.push("Serial Bus: Error accessing /dev/".to_string());
        }
        results
    }

    fn update_maps_list(&mut self) {
        self.available_maps.clear();
        let home = std::env::var("HOME").unwrap_or_else(|_| "/home/pidev".to_string());
        let maps_path = format!("{}/titan_ws/src/titan_bringup/maps/", home);
        if let Ok(entries) = std::fs::read_dir(maps_path) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().and_then(|s| s.to_str()) == Some("yaml") {
                    if let Some(file_name) = path.file_name().and_then(|s| s.to_str()) {
                        self.available_maps.push(file_name.to_string());
                    }
                }
            }
        }
        self.available_maps.sort();
        self.map_selection_index = 0;
    }

    fn update_rviz_configs_list(&mut self) {
        self.available_rviz_configs.clear();
        let home = std::env::var("HOME").unwrap_or_else(|_| "/home/pidev".to_string());
        let config_path = format!("{}/titan_ws/src/titan_bringup/rviz_config/", home);
        if let Ok(entries) = std::fs::read_dir(config_path) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().and_then(|s| s.to_str()) == Some("rviz") {
                    if let Some(file_name) = path.file_name().and_then(|s| s.to_str()) {
                        self.available_rviz_configs.push(file_name.to_string());
                    }
                }
            }
        }
        self.available_rviz_configs.sort();
        self.rviz_config_selection_index = 0;
    }

    fn update_wifi_list(&mut self) {
        self.is_loading = true;
        self.loading_message = "Scanning for WiFi networks...".to_string();
        self.logs.push("Refreshing WiFi list (USB Adapter)...".to_string());
        
        // Force a rescan first
        let _ = Command::new("nmcli")
            .args(["device", "wifi", "rescan", "ifname", "wlxd03745f25a51"])
            .output();

        let output = Command::new("nmcli")
            .args(["-t", "-f", "SSID,SIGNAL", "dev", "wifi", "list", "ifname", "wlxd03745f25a51"])
            .output();

        if let Ok(out) = output {
            let s = String::from_utf8_lossy(&out.stdout);
            let mut entries: Vec<(String, i32)> = s.lines()
                .filter(|line| !line.is_empty() && !line.starts_with("--"))
                .filter_map(|line| {
                    let parts: Vec<&str> = line.split(':').collect();
                    if parts.len() >= 2 {
                        let ssid = parts[0].to_string();
                        let signal = parts[1].parse::<i32>().unwrap_or(0);
                        Some((ssid, signal))
                    } else {
                        None
                    }
                })
                .collect();
            
            // Deduplicate by taking strongest signal
            entries.sort_by(|a, b| b.1.cmp(&a.1));
            let mut unique_ssids = Vec::new();
            let mut seen = std::collections::HashSet::new();
            for (ssid, signal) in entries {
                if !seen.contains(&ssid) {
                    seen.insert(ssid.clone());
                    unique_ssids.push(format!("{} ({}%)", ssid, signal));
                }
            }
            
            self.available_ssids = unique_ssids;
            
            if self.available_ssids.is_empty() {
                self.logs.push("No WiFi networks found.".to_string());
            } else {
                self.logs.push(format!("Found {} networks.", self.available_ssids.len()));
            }
        } else {
            self.logs.push("Error: Failed to scan for WiFi.".to_string());
        }
        self.is_loading = false;
    }

    #[allow(dead_code)]
    fn perform_wifi_connection(&mut self) {
        self.is_loading = true;
        self.loading_message = format!("Connecting to {}...", self.selected_ssid);
        
        let ssid_raw = self.selected_ssid.split(" (").next().unwrap_or(&self.selected_ssid).to_string();
        let pass = self.wifi_password_input.clone();
        self.logs.push(format!("Connecting to {}...", ssid_raw));
        
        let cmd = format!("echo \"pi@bin\" | sudo -S nmcli device wifi connect \"{}\" password \"{}\" ifname wlxd03745f25a51", ssid_raw, pass);
        let output = Command::new("bash").arg("-c").arg(cmd).output();
        
        match output {
            Ok(out) => {
                if out.status.success() {
                    self.logs.push(format!("Successfully connected to {}!", ssid_raw));
                    self.operation_status = "IDLE".to_string();
                    self.screen = Screen::Main;
                } else {
                    let err = String::from_utf8_lossy(&out.stderr);
                    self.logs.push(format!("Connection failed: {}", err));
                }
            },
            Err(e) => self.logs.push(format!("Execution error: {}", e)),
        }
        self.is_loading = false;
        self.wifi_password_input.clear();
    }

    fn on_tick(&mut self) {
        if self.telemetry_rx.has_changed().unwrap_or(false) {
            let data = self.telemetry_rx.borrow_and_update();
            self.current_telemetry = data.clone();
        }

        // Update spinner frame roughly every 100ms
        if self.last_tick.elapsed() >= Duration::from_millis(100) {
            self.spinner_frame = (self.spinner_frame + 1) % 8;
            self.last_tick = std::time::Instant::now();
        }

        // Check if active process is still running
        if let Some(ref mut child) = self.active_process {
            match child.try_wait() {
                Ok(Some(status)) => {
                    self.logs.push(format!("Process finished with status: {}", status));
                    self.active_process = None;
                    self.operation_status = "IDLE".to_string();
                },
                Ok(None) => {},
                Err(e) => {
                    self.logs.push(format!("Error checking process: {}", e));
                    self.active_process = None;
                    self.operation_status = "IDLE".to_string();
                }
            }
        }
    }

    fn translate_log(&self, msg: &str) -> String {
        if msg.contains("bringup.launch.py") { "Initializing Hardware Drivers...".to_string() }
        else if msg.contains("mapping.launch.py") { "Starting SLAM Toolbox Session...".to_string() }
        else if msg.contains("cartographer.launch.py") { "Starting Google Cartographer SLAM...".to_string() }
        else if msg.contains("navigation.launch.py") { "Activating Nav2 Stack...".to_string() }
        else if msg.contains("teleop_twist_keyboard") { "Opening Teleop Terminal...".to_string() }
        else if msg.contains("map_saver_cli") { "Compressing & Saving Map Data...".to_string() }
        else if msg.contains("colcon build") { "Compiling Workspace Packages...".to_string() }
        else if msg.contains("pkill -9 -f ros2") { "CRITICAL: PURGING ALL ROS2 PROCESSES...".to_string() }
        else { msg.to_string() }
    }

    fn execute_selected(&mut self) {
        let item = if self.screen == Screen::TeleopConfig {
            self.pending_teleop_item.clone().unwrap_or(MenuItem::LocalTeleop)
        } else {
            let menu_items = self.get_filtered_menu();
            if self.active_menu_index >= menu_items.len() { return; }
            menu_items[self.active_menu_index].clone()
        };

        if self.screen != Screen::TeleopConfig && (item == MenuItem::LocalTeleop || item == MenuItem::RemoteTeleop) {
            self.pending_teleop_item = Some(item);
            self.screen = Screen::TeleopConfig;
            self.teleop_field_index = 0;
            return;
        }

        if self.screen == Screen::TeleopConfig {
            self.screen = Screen::Main;
        }
        
        match item {
            MenuItem::Battery => {
                let status = self.get_battery_readable();
                self.logs.push(format!("Power Status: {}", status));
            },
            MenuItem::CheckUSB => {
                let usb = self.get_usb_readable();
                for res in usb {
                    self.logs.push(format!("USB Check: {}", res));
                }
            },
            MenuItem::Waypoints => {
            self.load_waypoints();
            self.screen = Screen::WaypointList;
            self.waypoint_selection_index = 0;
        },
            _ => {
                if self.screen != Screen::MapSelect && item == MenuItem::Navigation {
                    self.update_maps_list();
                    if self.available_maps.is_empty() {
                        self.logs.push("Error: No maps found in 'titan_bringup/maps/'!".to_string());
                        return;
                    }
                    self.screen = Screen::MapSelect;
                    return;
                }

                if self.screen != Screen::RVizConfigSelect && item == MenuItem::RemoteRViz {
                    self.update_rviz_configs_list();
                    if self.available_rviz_configs.is_empty() {
                        self.logs.push("Error: No RViz configs found in 'rviz_config/'!".to_string());
                        return;
                    }
                    self.screen = Screen::RVizConfigSelect;
                    return;
                }

                if self.screen == Screen::MapSelect || self.screen == Screen::RVizConfigSelect {
                    self.screen = Screen::Main;
                }

                self.logs.push(self.translate_log(&format!("Executing: {}", item.to_string())));
                match item {
                    MenuItem::Bringup => {
                        self.operation_status = "BRINGUP".to_string();
                        self.spawn_ros_launch("bringup.launch.py");
                    },
                    MenuItem::Mapping => {
                        self.operation_status = "MAPPING".to_string();
                        self.spawn_ros_launch("mapping.launch.py");
                    },
                    MenuItem::Cartographer => {
                        self.operation_status = "ADV_MAPPING".to_string();
                        self.spawn_ros_launch("cartographer.launch.py");
                    },
                    MenuItem::Navigation => {
                        if self.available_maps.is_empty() {
                            self.logs.push("Error: No maps available. Please run mapping first!".to_string());
                            return;
                        }
                        self.operation_status = "NAVIGATION".to_string();
                        let home = std::env::var("HOME").unwrap_or_else(|_| "/home/pidev".to_string());
                        let maps_path = format!("{}/titan_ws/src/titan_bringup/maps/", home);
                        let map_file = format!("{}{}", maps_path, self.available_maps[self.map_selection_index]);
                        let map_arg = format!("map:={}", map_file);
                        self.spawn_ros_launch_with_args("navigation.launch.py", vec![&map_arg]);
                    },
                    MenuItem::LocalTeleop => {
                        let is_tmux = std::env::var("TMUX").is_ok();
                        let speed = if self.teleop_speed.is_empty() { "0.2" } else { &self.teleop_speed };
                        let turn = if self.teleop_turn.is_empty() { "0.8" } else { &self.teleop_turn };
                        let ros_args = format!("--ros-args -p speed:={} -p turn:={}", speed, turn);
                        
                        if is_tmux {
                            self.logs.push(format!("Spawning Teleop (s={}, t={}) in split...", speed, turn));
                            let tmux_cmd = format!("ros2 run teleop_twist_keyboard teleop_twist_keyboard {}", ros_args);
                            let _ = Command::new("tmux")
                                .args(["split-window", "-h", &tmux_cmd])
                                .stdout(Stdio::null())
                                .stderr(Stdio::null())
                                .spawn();
                        } else {
                            self.logs.push("Error: Local Teleop requires tmux split!".to_string());
                        }
                    },
                    MenuItem::RemoteTeleop => {
                        let speed = if self.teleop_speed.is_empty() { "0.2" } else { &self.teleop_speed };
                        let turn = if self.teleop_turn.is_empty() { "0.8" } else { &self.teleop_turn };
                        let ros_args = format!("--ros-args -p speed:={} -p turn:={}", speed, turn);

                        self.logs.push(format!("Spawning Remote Teleop (s={}, t={}) in new window...", speed, turn));
                        let cmd_str = format!("gnome-terminal -- bash -c 'ros2 run teleop_twist_keyboard teleop_twist_keyboard {}; exec bash'", ros_args);
                        let _ = Command::new("bash")
                            .arg("-c")
                            .arg(cmd_str)
                            .stdout(Stdio::null())
                            .stderr(Stdio::null())
                            .spawn()
                            .map(|child| self.active_process = Some(child));
                    },
                    MenuItem::RemoteRViz => {
                        let config_file = if self.available_rviz_configs.is_empty() {
                            "titan.rviz".to_string()
                        } else {
                            self.available_rviz_configs[self.rviz_config_selection_index].clone()
                        };
                        self.logs.push(format!("Spawning Remote RViz with config: {}...", config_file));
                        let cmd_str = format!("gnome-terminal -- bash -c 'LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d ~/titan_ws/src/titan_bringup/rviz_config/{}; exec bash'", config_file);
                        let _ = Command::new("bash")
                            .arg("-c")
                            .arg(cmd_str)
                            .stdout(Stdio::null())
                            .stderr(Stdio::null())
                            .spawn()
                            .map(|child| self.active_process = Some(child));
                    },
                    MenuItem::SaveMap => {
                        self.screen = Screen::MapNameInput;
                        self.map_name_input.clear();
                    },
                    MenuItem::ConnectWiFi => {
                        self.update_wifi_list();
                        if !self.available_ssids.is_empty() {
                            self.screen = Screen::WifiScan;
                            self.wifi_selection_index = 0;
                        }
                    },
                    MenuItem::Rebuild => {
                        self.is_loading = true;
                        self.loading_message = "Rebuilding Workspace...".to_string();
                        self.logs.push("Rebuilding...".to_string());
                        let home = std::env::var("HOME").unwrap_or_else(|_| "/home/pidev".to_string());
                        let build_cmd = format!("cd {}/titan_ws && colcon build --symlink-install", home);
                        let output = Command::new("bash").arg("-c").arg(build_cmd).output();
                        self.handle_output(output, "Rebuild complete.");
                        self.is_loading = false;
                    },
                    MenuItem::KillAll => {
                        self.logs.push(self.translate_log("pkill -9 -f ros2"));
                        let kill_cmd = "pkill -9 -f ros2; pkill -9 -f ydlidar; pkill -9 -f slam_toolbox; pkill -9 -f arduino_bridge; ros2 daemon stop; ros2 daemon start";
                        let _ = Command::new("bash").arg("-c").arg(kill_cmd).output();
                        self.logs.push("Global Reset Complete.".to_string());
                        self.operation_status = "IDLE".to_string();
                        self.active_process = None;
                    },
                    _ => {}
                }
            }
        }
    }

    fn spawn_ros_launch(&mut self, file: &str) {
        self.spawn_ros_launch_with_args(file, Vec::new());
    }

    fn spawn_ros_launch_with_args(&mut self, file: &str, args: Vec<&str>) {
        let args_str = args.join(" ");
        let cmd_str = format!("source ~/titan_ws/install/setup.bash && ros2 launch titan_bringup {} {}", file, args_str);

        if self.device_type == DeviceType::Laptop {
            self.logs.push(format!("Laptop Mode: {} launching in new window...", file));
            let terminal_cmd = format!("gnome-terminal -- bash -c \"{}; exec bash\"", cmd_str);
            let _ = Command::new("bash")
                .arg("-c")
                .arg(&terminal_cmd)
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .spawn()
                .map(|child| self.active_process = Some(child));
            return;
        }

        let is_tmux = std::env::var("TMUX").is_ok();
        if is_tmux {
            self.logs.push("Spawning in tmux split...".to_string());
            let _ = Command::new("tmux")
                .args(["split-window", "-h", &cmd_str])
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .spawn();
        } else {
            if self.active_process.is_some() {
                if let Some(mut child) = self.active_process.take() {
                    let _ = child.kill();
                }
            }
            let _ = Command::new("bash")
                .arg("-c")
                .arg(&cmd_str)
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .spawn()
                .map(|child| self.active_process = Some(child));
        }
    }

    fn handle_output(&mut self, res: io::Result<std::process::Output>, success_msg: &str) {
        match res {
            Ok(out) => {
                if out.status.success() {
                    self.logs.push(self.translate_log(success_msg));
                    let stdout = String::from_utf8_lossy(&out.stdout);
                    let lines: Vec<&str> = stdout.lines()
                        .filter(|l| !l.trim().is_empty() && !l.contains("total 0"))
                        .collect();
                    let start = lines.len().saturating_sub(15);
                    for line in &lines[start..] {
                        self.logs.push(format!("  {}", line));
                    }
                } else {
                    self.logs.push(format!("Failed: {}", String::from_utf8_lossy(&out.stderr)));
                }
            },
            Err(e) => self.logs.push(format!("Error: {}", e)),
        }
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let ctx = r2r::Context::create().expect("Failed to create ROS 2 context");
    let mut node = r2r::Node::create(ctx, "titan_tui", "").expect("Failed to create ROS 2 node");
    
    let (tx, rx) = watch::channel(Telemetry::default());
    let (cmd_tx, mut cmd_rx) = mpsc::unbounded_channel::<RosCommand>();

    // Background ROS task
    tokio::spawn(async move {
        let mut sub = node.subscribe::<Odometry>("/odom", r2r::QosProfile::default()).expect("Failed to subscribe to /odom");
        let goal_pub = node.create_publisher::<PoseStamped>("/goal_pose", r2r::QosProfile::default()).expect("Failed to create /goal_pose pub");
        let init_pub = node.create_publisher::<PoseWithCovarianceStamped>("/initialpose", r2r::QosProfile::default()).expect("Failed to create /initialpose pub");
        let reset_pub = node.create_publisher::<EmptyMsg>("/reset_odom", r2r::QosProfile::default()).expect("Failed to create /reset_odom pub");

        loop {
            node.spin_once(std::time::Duration::from_millis(10));
            // Telemetry
            match tokio::time::timeout(std::time::Duration::from_millis(5), sub.next()).await {
                Ok(Some(msg)) => {
                    let x = msg.pose.pose.position.x;
                    let y = msg.pose.pose.position.y;
                    let q = msg.pose.pose.orientation;
                    let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
                    let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
                    let theta = f64::atan2(siny_cosp, cosy_cosp);
                    let stamp = msg.header.stamp.clone();
                    let _ = tx.send(Telemetry { x, y, theta, stamp });
                }
                _ => {}
            }

            // Commands
            while let Ok(cmd) = cmd_rx.try_recv() {
                match cmd {
                    RosCommand::NavTo(wp) => {
                        let mut msg = PoseStamped::default();
                        msg.header.frame_id = "map".to_string();
                        msg.header.stamp = tx.borrow().stamp.clone();
                        msg.pose.position.x = wp.x;
                        msg.pose.position.y = wp.y;
                        msg.pose.orientation = euler_to_quaternion(wp.theta);
                        let _ = goal_pub.publish(&msg);
                    },
                    RosCommand::SetPose(wp) => {
                        let mut msg = PoseWithCovarianceStamped::default();
                        msg.header.frame_id = "map".to_string();
                        msg.header.stamp = tx.borrow().stamp.clone();
                        msg.pose.pose.position.x = wp.x;
                        msg.pose.pose.position.y = wp.y;
                        msg.pose.pose.orientation = euler_to_quaternion(wp.theta);
                        // Standard initial pose covariance
                        msg.pose.covariance[0] = 0.25;
                        msg.pose.covariance[7] = 0.25;
                        msg.pose.covariance[35] = 0.06;
                        let _ = init_pub.publish(&msg);
                    },
                    RosCommand::ResetOdom => {
                        let msg = EmptyMsg::default();
                        let _ = reset_pub.publish(&msg);
                    }
                }
            }

            tokio::time::sleep(std::time::Duration::from_millis(10)).await;
        }
    });

    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut app = App::new(rx, cmd_tx);
    let res = run_app(&mut terminal, &mut app).await;

    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen, DisableMouseCapture)?;
    terminal.show_cursor()?;

    if let Err(err) = res { println!("{:?}", err) }
    Ok(())
}

fn euler_to_quaternion(yaw: f64) -> Quaternion {
    Quaternion {
        x: 0.0,
        y: 0.0,
        z: (yaw * 0.5).sin(),
        w: (yaw * 0.5).cos(),
    }
}

async fn run_app<B: ratatui::backend::Backend>(
    terminal: &mut Terminal<B>,
    app: &mut App,
) -> io::Result<()> {
    loop {
        terminal.draw(|f| {
            let size = f.size();
            
            if app.is_loading {
                render_loader(f, &app.loading_message, app.spinner_frame);
            }

            if app.screen == Screen::Banner {
                let main_block = Block::default()
                    .borders(Borders::ALL)
                    .border_type(BorderType::Thick)
                    .border_style(Style::default().fg(Color::Cyan))
                    .title(" [ TITAN CONTROL CENTER ] ")
                    .title_alignment(Alignment::Center);
                f.render_widget(main_block, size);

                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Length((size.height as i32 - 24).max(0) as u16 / 2),
                        Constraint::Length(8), // TRIDENT ASCII
                        Constraint::Length(1), // Subtitle
                        Constraint::Length(2), // Spacer
                        Constraint::Length(2), // Description
                        Constraint::Length(2), // Spacer
                        Constraint::Length(1), // Version
                        Constraint::Min(0),    // Hint
                    ].as_ref())
                    .split(size);

                let ascii_trident = r#"
 ████████╗██████╗ ██╗██████╗ ███████╗███╗   ██╗████████╗
 ╚══██╔══╝██╔══██╗██║██╔══██╗██╔════╝████╗  ██║╚══██╔══╝
    ██║   ██████╔╝██║██║  ██║█████╗  ██╔██╗ ██║   ██║   
    ██║   ██╔══██╗██║██║  ██║██╔══╝  ██║╚██╗██║   ██║   
    ██║   ██║  ██║██║██████╔╝███████╗██║ ╚████║   ██║   
    ╚═╝   ╚═╝  ╚═╝╚═╝╚═════╝ ╚══════╝╚═╝  ╚═══╝   ╚═╝   
                "#;
                let title = Paragraph::new(ascii_trident)
                    .style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD))
                    .alignment(Alignment::Center);
                
                let subtitle = Paragraph::new("Tactical Remote Interface for Detailed Exploration and Navigation of TITAN")
                    .alignment(Alignment::Center);

                let description_text = vec![
                    Line::from(vec![
                        Span::raw("A unified "),
                        Span::styled("Tactical Interface", Style::default().fg(Color::Magenta).add_modifier(Modifier::BOLD)),
                        Span::raw(" designed for "),
                    ]),
                    Line::from(vec![
                        Span::styled("Real-time Telemetry Control", Style::default().fg(Color::Yellow)),
                        Span::raw(" and "),
                        Span::styled("Autonomous Mission Flow", Style::default().fg(Color::Yellow)),
                    ]),
                ];
                let description = Paragraph::new(description_text).alignment(Alignment::Center);

                let metadata = Paragraph::new("v0.2.0 | Target: ROS2 Jazzy | Linux").alignment(Alignment::Center);
                let hint = Paragraph::new("\n\nPress [ENTER] to continue | [q] to quit")
                    .style(Style::default().fg(Color::DarkGray).add_modifier(Modifier::BOLD))
                    .alignment(Alignment::Center);

                f.render_widget(title, chunks[1]);
                f.render_widget(subtitle, chunks[2]);
                f.render_widget(description, chunks[4]); 
                f.render_widget(metadata, chunks[6]);
                f.render_widget(hint, chunks[7]);

            } else if app.screen == Screen::Splash {
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Length(3),
                        Constraint::Length(3),
                        Constraint::Min(0),
                        Constraint::Length(3),
                        Constraint::Length(3),
                    ].as_ref())
                    .split(size);

                let title = Paragraph::new("TRIDENT CONTROL SYSTEM v0.2.0")
                    .style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD))
                    .alignment(Alignment::Center)
                    .block(Block::default().borders(Borders::ALL));
                
                let checks: Vec<ListItem> = app.startup_checks.iter()
                    .map(|s| {
                        let color = if s.contains("NOT FOUND") || s.contains("DISCONNECTED") || s.contains("CRITICAL") { Color::Red }
                                   else if s.contains("Warning") || s.contains("LOW") { Color::Yellow }
                                   else if s.contains("OK") || s.contains("CONNECTED") || s.contains("STABLE") { Color::Green }
                                   else { Color::Gray };
                        ListItem::new(s.as_str()).style(Style::default().fg(color))
                    }).collect();
                
                let check_list = List::new(checks).block(Block::default().borders(Borders::ALL).title(" SYSTEM DIAGNOSTICS "));
                
                let elapsed = app.splash_start.elapsed().as_secs_f32();
                let progress = (elapsed / 3.0).min(1.0);
                let gauge = Gauge::default()
                    .block(Block::default().borders(Borders::ALL).title(" INITIALIZING "))
                    .gauge_style(Style::default().fg(Color::Cyan))
                    .ratio(progress as f64);

                f.render_widget(title, chunks[1]);
                f.render_widget(check_list, chunks[2]);
                f.render_widget(gauge, chunks[3]);

                if progress >= 1.0 { app.screen = Screen::Main; }

            } else if app.screen == Screen::DeviceSelect {
                let area = centered_rect(60, 30, size);
                let block = Block::default().borders(Borders::ALL).title(" DEVICE SELECTION ").border_style(Style::default().fg(Color::Cyan));
                let items = vec![
                    ListItem::new("  1. TITAN (Robot) - Controller Dashboard "),
                    ListItem::new("  2. LAPTOP (Remote) - Teleop & Monitoring "),
                ];
                let list = List::new(items).block(block).highlight_style(Style::default().fg(Color::Yellow));
                let mut state = ratatui::widgets::ListState::default();
                state.select(Some(app.selection_index));
                f.render_stateful_widget(list, area, &mut state);

            } else if app.screen == Screen::MapNameInput {
                let area = centered_rect(60, 20, size);
                let input = Paragraph::new(format!(" Name: {}_", app.map_name_input))
                    .block(Block::default().borders(Borders::ALL).title(" SAVE MAP AS... "));
                f.render_widget(input, area);

            } else if app.screen == Screen::TeleopConfig {
                let area = centered_rect(50, 35, size);
                let block = Block::default().borders(Borders::ALL).title(" TELEOP CONFIGURATION ");
                let inner = Layout::default().direction(Direction::Vertical).constraints([Constraint::Length(3), Constraint::Length(3), Constraint::Min(0)]).margin(2).split(area);
                let speed_style = if app.teleop_field_index == 0 { Style::default().fg(Color::Yellow) } else { Style::default() };
                let turn_style = if app.teleop_field_index == 1 { Style::default().fg(Color::Yellow) } else { Style::default() };
                let speed_field = Paragraph::new(format!(" Speed: {}_", app.teleop_speed)).block(Block::default().borders(Borders::ALL).title(" LINEAR ").border_style(speed_style));
                let turn_field = Paragraph::new(format!(" Turn: {}_", app.teleop_turn)).block(Block::default().borders(Borders::ALL).title(" ANGULAR ").border_style(turn_style));
                f.render_widget(block, area);
                f.render_widget(speed_field, inner[0]);
                f.render_widget(turn_field, inner[1]);

            } else if app.screen == Screen::MapSelect || app.screen == Screen::RVizConfigSelect {
                let area = centered_rect(40, 50, size);
                let title = if app.screen == Screen::MapSelect { " SELECT MAP " } else { " SELECT RVIZ CONFIG " };
                let items: Vec<ListItem> = (if app.screen == Screen::MapSelect { &app.available_maps } else { &app.available_rviz_configs })
                    .iter().enumerate().map(|(i, m)| {
                        let style = if i == (if app.screen == Screen::MapSelect { app.map_selection_index } else { app.rviz_config_selection_index }) {
                            Style::default().fg(Color::Yellow)
                        } else { Style::default().fg(Color::White) };
                        ListItem::new(m.as_str()).style(style)
                    }).collect();
                let list = List::new(items).block(Block::default().borders(Borders::ALL).title(title));
                f.render_widget(list, area);

            } else if app.screen == Screen::WifiScan {
                let area = centered_rect(50, 60, size);
                let items: Vec<ListItem> = app.available_ssids.iter().enumerate().map(|(i, s)| {
                    let style = if i == app.wifi_selection_index { Style::default().fg(Color::Yellow) } else { Style::default().fg(Color::White) };
                    ListItem::new(s.as_str()).style(style)
                }).collect();
                let list = List::new(items).block(Block::default().borders(Borders::ALL).title(" WIFI SCAN "));
                f.render_widget(list, area);

            } else if app.screen == Screen::WifiPasswordInput {
                let area = centered_rect(60, 20, size);
                let asterisks = "*".repeat(app.wifi_password_input.len());
                let input = Paragraph::new(format!(" Password: {}_", asterisks)).block(Block::default().borders(Borders::ALL).title(" WIFI PASSWORD "));
                f.render_widget(input, area);

            } else if app.screen == Screen::WaypointList {
                let area = centered_rect(60, 70, size);
                let mut items: Vec<ListItem> = app.waypoints.iter().enumerate().map(|(i, wp)| {
                    let style = if i == app.waypoint_selection_index { Style::default().fg(Color::Yellow) } else { Style::default().fg(Color::White) };
                    ListItem::new(format!("  {} ({:.2}, {:.2})", wp.name, wp.x, wp.y)).style(style)
                }).collect();
                items.push(ListItem::new("  [+] SAVE CURRENT POSITION ").style(Style::default().fg(Color::Green)));
                let list = List::new(items).block(Block::default().borders(Borders::ALL).title(" WAYPOINTS - [ENTER] to Nav | [P] to Init Pose | [DEL] to Remove "));
                f.render_widget(list, area);

            } else if app.screen == Screen::WaypointNameInput {
                let area = centered_rect(60, 20, size);
                let input = Paragraph::new(format!(" Name: {}_", app.waypoint_name_input))
                    .block(Block::default().borders(Borders::ALL).title(" NAME NEW WAYPOINT "));
                f.render_widget(input, area);

            } else {
                let chunks = Layout::default().direction(Direction::Vertical).constraints([Constraint::Length(3), Constraint::Min(0), Constraint::Length(3)]).split(size);
                let body_chunks = Layout::default().direction(Direction::Horizontal).constraints([Constraint::Percentage(33), Constraint::Percentage(27), Constraint::Percentage(40)]).split(chunks[1]);

                let header = Paragraph::new("TRIDENT v0.2.0")
                    .alignment(Alignment::Center)
                    .block(Block::default().borders(Borders::ALL).border_style(Style::default().fg(Color::Cyan)));
                
                let menu_items: Vec<ListItem> = app.get_filtered_menu().iter().enumerate().map(|(i, item)| {
                    let style = if i == app.active_menu_index { Style::default().fg(Color::Yellow).bg(Color::Rgb(40,40,40)) } else { Style::default().fg(Color::White) };
                    ListItem::new(format!("  {}", item.to_string())).style(style)
                }).collect();
                let menu_list = List::new(menu_items).block(Block::default().borders(Borders::ALL).title(" OPERATIONS "));

                let t = &app.current_telemetry;
                let odom_text = vec![
                    Line::from(vec![Span::styled(" [X] ", Style::default().fg(Color::Cyan)), Span::raw(format!("{:.3} m", t.x))]),
                    Line::from(vec![Span::styled(" [Y] ", Style::default().fg(Color::Cyan)), Span::raw(format!("{:.3} m", t.y))]),
                    Line::from(vec![Span::styled(" [θ] ", Style::default().fg(Color::Cyan)), Span::raw(format!("{:.3} rad", t.theta))]),
                ];
                let odom_panel = Paragraph::new(odom_text).block(Block::default().borders(Borders::ALL).title(" TELEMETRY "));

                let display_logs = if app.logs.len() > 15 { &app.logs[app.logs.len()-15..] } else { &app.logs[..] };
                let logs: Vec<ListItem> = display_logs.iter().map(|s| {
                    let color = if s.contains("Error") { Color::Red } else if s.contains("complete") { Color::Green } else { Color::Gray };
                    ListItem::new(format!("> {}", s)).style(Style::default().fg(color))
                }).collect();
                let logs_panel = List::new(logs).block(Block::default().borders(Borders::ALL).title(" SYSTEM LOGS "));

                let footer = Paragraph::new(" role: ".to_owned() + app.device_type.to_string() + " | arrows: select | enter: exec | c: clear | r: reset odom | q: quit")
                    .alignment(Alignment::Center).block(Block::default().borders(Borders::ALL));

                f.render_widget(header, chunks[0]);
                f.render_widget(menu_list, body_chunks[0]);
                f.render_widget(odom_panel, body_chunks[1]);
                f.render_widget(logs_panel, body_chunks[2]);
                f.render_widget(footer, chunks[2]);
            }
        })?;

        if crossterm::event::poll(Duration::from_millis(100))? {
            if let Event::Key(key) = event::read()? {
                if app.screen == Screen::Banner {
                    match key.code { KeyCode::Enter => app.screen = Screen::DeviceSelect, KeyCode::Char('q') => return Ok(()), _ => {} }
                } else if app.screen == Screen::DeviceSelect {
                    match key.code {
                        KeyCode::Up => if app.selection_index > 0 { app.selection_index -= 1; },
                        KeyCode::Down => if app.selection_index < 1 { app.selection_index += 1; },
                        KeyCode::Enter => {
                            app.device_type = if app.selection_index == 0 { DeviceType::Titan } else { DeviceType::Laptop };
                            app.run_diagnostics(); app.splash_start = std::time::Instant::now(); app.screen = Screen::Splash;
                        },
                        _ => {}
                    }
                } else if app.screen == Screen::TeleopConfig {
                    match key.code {
                        KeyCode::Esc => app.screen = Screen::Main,
                        KeyCode::Up | KeyCode::Down => {
                            app.teleop_field_index = if app.teleop_field_index == 0 { 1 } else { 0 };
                        },
                        KeyCode::Backspace => {
                            if app.teleop_field_index == 0 { app.teleop_speed.pop(); }
                            else { app.teleop_turn.pop(); }
                        },
                        KeyCode::Char(c) => {
                            if c.is_digit(10) || c == '.' {
                                if app.teleop_field_index == 0 { app.teleop_speed.push(c); }
                                else { app.teleop_turn.push(c); }
                            }
                        },
                        KeyCode::Enter => {
                            app.execute_selected();
                        },
                        _ => {}
                    }
                } else if app.screen == Screen::MapNameInput {
                    match key.code {
                        KeyCode::Esc => app.screen = Screen::Main,
                        KeyCode::Backspace => { app.map_name_input.pop(); },
                        KeyCode::Char(c) => { app.map_name_input.push(c); },
                        KeyCode::Enter => {
                            if !app.map_name_input.is_empty() {
                                let name = app.map_name_input.clone();
                                let path = format!("/home/pidev/titan_ws/src/titan_bringup/maps/{}", name);
                                let output = Command::new("ros2")
                                    .args(["run", "nav2_map_server", "map_saver_cli", "-f", &path])
                                    .output();
                                
                                if let Ok(ref out) = output {
                                    if out.status.success() {
                                        let pgm_path = format!("{}.pgm", path);
                                        let _ = Command::new("python3")
                                            .args(["/home/pidev/titan_tui/scripts/clean_map.py", &pgm_path])
                                            .output();
                                        app.handle_output(output, &format!("Map {} saved and post-processed.", name));
                                    } else {
                                        app.handle_output(output, &format!("Failed to save map {}.", name));
                                    }
                                } else {
                                    app.handle_output(output, &format!("Failed to execute map_saver for {}.", name));
                                }
                                app.screen = Screen::Main;
                            }
                        }, _ => {}
                    }
                } else if app.screen == Screen::WaypointList {
                    match key.code {
                        KeyCode::Esc => app.screen = Screen::Main,
                        KeyCode::Up => if app.waypoint_selection_index > 0 { app.waypoint_selection_index -= 1; },
                        KeyCode::Down => if app.waypoint_selection_index < app.waypoints.len() { app.waypoint_selection_index += 1; },
                        KeyCode::Enter => {
                            if app.waypoint_selection_index == app.waypoints.len() {
                                app.screen = Screen::WaypointNameInput;
                                app.waypoint_name_input.clear();
                            } else {
                                let wp = app.waypoints[app.waypoint_selection_index].clone();
                                let _ = app.ros_cmd_tx.send(RosCommand::NavTo(wp));
                                app.logs.push(format!("Sending Nav goal to: {}", app.waypoints[app.waypoint_selection_index].name));
                                app.screen = Screen::Main;
                            }
                        },
                        KeyCode::Char('p') | KeyCode::Char('P') => {
                            if app.waypoint_selection_index < app.waypoints.len() {
                                let wp = app.waypoints[app.waypoint_selection_index].clone();
                                let _ = app.ros_cmd_tx.send(RosCommand::SetPose(wp));
                                app.logs.push(format!("Initialized Pose at: {}", app.waypoints[app.waypoint_selection_index].name));
                                app.screen = Screen::Main;
                            }
                        },
                        KeyCode::Char('s') | KeyCode::Char('S') => {
                            app.screen = Screen::WaypointNameInput;
                            app.waypoint_name_input.clear();
                        },
                        KeyCode::Delete => {
                            if app.waypoint_selection_index < app.waypoints.len() {
                                app.waypoints.remove(app.waypoint_selection_index);
                                app.save_waypoints();
                            }
                        },
                        KeyCode::Char('c') | KeyCode::Char('C') => {
                            app.logs.clear();
                        },
                        _ => {}
                    }
                } else if app.screen == Screen::WaypointNameInput {
                    match key.code {
                        KeyCode::Esc => app.screen = Screen::WaypointList,
                        KeyCode::Backspace => { app.waypoint_name_input.pop(); },
                        KeyCode::Char(c) => { app.waypoint_name_input.push(c); },
                        KeyCode::Enter => {
                            if !app.waypoint_name_input.is_empty() {
                                let wp = Waypoint {
                                    name: app.waypoint_name_input.clone(),
                                    x: app.current_telemetry.x,
                                    y: app.current_telemetry.y,
                                    theta: app.current_telemetry.theta,
                                };
                                app.waypoints.push(wp);
                                app.save_waypoints();
                                app.screen = Screen::WaypointList;
                            }
                        }, _ => {}
                    }
                } else if app.screen == Screen::MapSelect {
                    match key.code {
                        KeyCode::Esc => app.screen = Screen::Main,
                        KeyCode::Up => if app.map_selection_index > 0 { app.map_selection_index -= 1; },
                        KeyCode::Down => if app.map_selection_index < app.available_maps.len() - 1 { app.map_selection_index += 1; },
                        KeyCode::Enter => app.execute_selected(),
                        _ => {}
                    }
                } else if app.screen == Screen::RVizConfigSelect {
                    match key.code {
                        KeyCode::Esc => app.screen = Screen::Main,
                        KeyCode::Up => if app.rviz_config_selection_index > 0 { app.rviz_config_selection_index -= 1; },
                        KeyCode::Down => if app.rviz_config_selection_index < app.available_rviz_configs.len() - 1 { app.rviz_config_selection_index += 1; },
                        KeyCode::Enter => app.execute_selected(),
                        _ => {}
                    }
                } else if app.screen == Screen::WifiScan {
                    match key.code {
                        KeyCode::Esc => app.screen = Screen::Main,
                        KeyCode::Up => if app.wifi_selection_index > 0 { app.wifi_selection_index -= 1; },
                        KeyCode::Down => if app.wifi_selection_index < app.available_ssids.len() - 1 { app.wifi_selection_index += 1; },
                        KeyCode::Enter => {
                            app.selected_ssid = app.available_ssids[app.wifi_selection_index].clone();
                            app.screen = Screen::WifiPasswordInput;
                            app.wifi_password_input.clear();
                        },
                        _ => {}
                    }
                } else if app.screen == Screen::WifiPasswordInput {
                    match key.code {
                        KeyCode::Esc => app.screen = Screen::WifiScan,
                        KeyCode::Backspace => { app.wifi_password_input.pop(); },
                        KeyCode::Char(c) => { app.wifi_password_input.push(c); },
                        KeyCode::Enter => {
                            app.perform_wifi_connection();
                            app.screen = Screen::Main;
                        },
                        _ => {}
                    }
                } else if app.screen == Screen::Main {
                    match key.code {
                        KeyCode::Up => if app.active_menu_index > 0 { app.active_menu_index -= 1; },
                        KeyCode::Down => if app.active_menu_index < app.get_filtered_menu().len()-1 { app.active_menu_index += 1; },
                        KeyCode::Enter => app.execute_selected(),
                        KeyCode::Char('c') | KeyCode::Char('C') => {
                            app.logs.clear();
                        },
                        KeyCode::Char('r') | KeyCode::Char('R') => {
                            let _ = app.ros_cmd_tx.send(RosCommand::ResetOdom);
                            app.logs.push("Sent Odometry Reset command.".to_string());
                        },
                        KeyCode::Char('q') => return Ok(()),
                        _ => {}
                    }
                } else {
                     match key.code { KeyCode::Esc => app.screen = Screen::Main, _ => {} }
                }
            }
        }
        app.on_tick();
    }
}

fn centered_rect(percent_x: u16, percent_y: u16, r: Rect) -> Rect {
    let popup_layout = Layout::default().direction(Direction::Vertical).constraints([Constraint::Percentage((100 - percent_y) / 2), Constraint::Percentage(percent_y), Constraint::Percentage((100 - percent_y) / 2)]).split(r);
    Layout::default().direction(Direction::Horizontal).constraints([Constraint::Percentage((100 - percent_x) / 2), Constraint::Percentage(percent_x), Constraint::Percentage((100 - percent_x) / 2)]).split(popup_layout[1])[1]
}

fn render_loader(f: &mut ratatui::Frame, message: &str, frame: usize) {
    let area = centered_rect(60, 20, f.size());
    let frames = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧"];
    let text = vec![Line::from(format!(" {} {}", frames[frame % 8], message)), Line::from(" Please wait... ")];
    let paragraph = Paragraph::new(text).block(Block::default().borders(Borders::ALL).border_style(Style::default().fg(Color::Yellow))).alignment(Alignment::Center);
    f.render_widget(ratatui::widgets::Clear, area);
    f.render_widget(paragraph, area);
}
