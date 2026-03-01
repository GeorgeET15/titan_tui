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
use std::{io, time::Duration, process::{Command, Child}};
use futures::StreamExt;
use r2r::nav_msgs::msg::Odometry;
use tokio::sync::watch;

#[derive(Clone, Default)]
struct Telemetry {
    x: f64,
    y: f64,
    theta: f64,
}

#[derive(Clone, PartialEq, Eq)]
enum MenuItem {
    // Robot Local
    Bringup,
    LocalTeleop,
    Mapping,
    Navigation,
    SaveMap,
    // System
    CheckUSB,
    Battery,
    Rebuild,
    StopProcess,
    // Laptop Remote (Instructions)
    RemoteTeleop,
    RemoteRViz,
}
impl MenuItem {
    fn to_string(&self) -> String {
        match self {
            MenuItem::Bringup => "[Robot] START BRINGUP".to_string(),
            MenuItem::LocalTeleop => "[Robot] LOCAL TELEOP (TMUX)".to_string(),
            MenuItem::Mapping => "[Robot] START MAPPING".to_string(),
            MenuItem::Navigation => "[Robot] START NAV2".to_string(),
            MenuItem::SaveMap => "[Robot] SAVE MAP".to_string(),
            MenuItem::CheckUSB => "[Sys] CHECK USB/SERIAL".to_string(),
            MenuItem::Battery => "[Sys] POWER/BATTERY".to_string(),
            MenuItem::Rebuild => "[Sys] REBUILD WORKSPACE".to_string(),
            MenuItem::StopProcess => "[Sys] STOP BACKGROUND PROC".to_string(),
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
}

impl App {
    fn new(telemetry_rx: watch::Receiver<Telemetry>) -> App {
        App {
            logs: vec![
                "TITAN System initialized".to_string(),
                "Ready for commands".to_string(),
            ],
            all_menu_items: vec![
                MenuItem::Bringup,
                MenuItem::LocalTeleop,
                MenuItem::Mapping,
                MenuItem::Navigation,
                MenuItem::SaveMap,
                MenuItem::CheckUSB,
                MenuItem::Battery,
                MenuItem::Rebuild,
                MenuItem::StopProcess,
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
            teleop_turn: "0.7".to_string(),
            teleop_field_index: 0,
            pending_teleop_item: None,
            available_maps: Vec::new(),
            map_selection_index: 0,
            available_rviz_configs: Vec::new(),
            rviz_config_selection_index: 0,
            operation_status: "IDLE".to_string(),
            spinner_frame: 0,
            last_tick: std::time::Instant::now(),
        }
    }

    fn get_filtered_menu(&self) -> Vec<MenuItem> {
        self.all_menu_items.iter().filter(|item| {
            match (self.device_type, item) {
                (DeviceType::Titan, MenuItem::RemoteTeleop) | (DeviceType::Titan, MenuItem::RemoteRViz) => false,
                (DeviceType::Laptop, MenuItem::Bringup) | (DeviceType::Laptop, MenuItem::LocalTeleop) | 
                (DeviceType::Laptop, MenuItem::Mapping) | (DeviceType::Laptop, MenuItem::Navigation) |
                (DeviceType::Laptop, MenuItem::SaveMap) | (DeviceType::Laptop, MenuItem::CheckUSB) |
                (DeviceType::Laptop, MenuItem::Battery) | (DeviceType::Laptop, MenuItem::Rebuild) |
                (DeviceType::Laptop, MenuItem::StopProcess) => false,
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
                Ok(None) => {
                    // Still running
                },
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
        else if msg.contains("mapping.launch.py") { "Starting SLAM (Mapping) Session...".to_string() }
        else if msg.contains("navigation.launch.py") { "Activating Nav2 Stack...".to_string() }
        else if msg.contains("teleop_twist_keyboard") { "Opening Teleop Terminal...".to_string() }
        else if msg.contains("map_saver_cli") { "Compressing & Saving Map Data...".to_string() }
        else if msg.contains("colcon build") { "Compiling Workspace Packages...".to_string() }
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
                        let _ = Command::new("bash").arg("-c").arg(cmd_str).spawn().map(|child| self.active_process = Some(child));
                    },
                    MenuItem::RemoteRViz => {
                        let config_file = if self.available_rviz_configs.is_empty() {
                            "titan.rviz".to_string()
                        } else {
                            self.available_rviz_configs[self.rviz_config_selection_index].clone()
                        };
                        self.logs.push(format!("Spawning Remote RViz with config: {}...", config_file));
                        let cmd_str = format!("gnome-terminal -- bash -c 'rviz2 -d ~/titan_ws/src/titan_bringup/rviz_config/{}; exec bash'", config_file);
                        let _ = Command::new("bash").arg("-c").arg(cmd_str).spawn().map(|child| self.active_process = Some(child));
                    },
                    MenuItem::SaveMap => {
                        self.screen = Screen::MapNameInput;
                        self.map_name_input.clear();
                    },
                    MenuItem::Rebuild => {
                        self.logs.push("Rebuilding...".to_string());
                        let home = std::env::var("HOME").unwrap_or_else(|_| "/home/pidev".to_string());
                        let build_cmd = format!("cd {}/titan_ws && colcon build --symlink-install", home);
                        let output = Command::new("bash").arg("-c").arg(build_cmd).output();
                        self.handle_output(output, "Rebuild complete.");
                    },
                    MenuItem::StopProcess => {
                        if let Some(mut child) = self.active_process.take() {
                            let _ = child.kill();
                            self.logs.push("Background process terminated.".to_string());
                        } else {
                            self.logs.push("No background process active.".to_string());
                        }
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
            let _ = Command::new("bash").arg("-c").arg(&terminal_cmd).spawn().map(|child| self.active_process = Some(child));
            return;
        }

        let is_tmux = std::env::var("TMUX").is_ok();
        if is_tmux {
            self.logs.push("Spawning in tmux split...".to_string());
            let _ = Command::new("tmux").args(["split-window", "-h", &cmd_str]).spawn();
        } else {
            if self.active_process.is_some() {
                if let Some(mut child) = self.active_process.take() {
                    let _ = child.kill();
                }
            }
            self.logs.push(self.translate_log(&format!("Launching: {}...", file)));
            let _ = Command::new("bash").arg("-c").arg(&cmd_str).spawn().map(|child| self.active_process = Some(child));
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
                    
                    // Show only the last 15 lines of output to avoid flooding
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
    // ROS 2 Initialization
    let ctx = r2r::Context::create().expect("Failed to create ROS 2 context");
    let mut node = r2r::Node::create(ctx, "titan_tui", "").expect("Failed to create ROS 2 node");
    
    let (tx, rx) = watch::channel(Telemetry::default());
    
    // Subscribe to Odometry
    let mut sub = node.subscribe::<Odometry>("/odom", r2r::QosProfile::default()).expect("Failed to subscribe to /odom");
    
    // Background ROS task (handles sub and spin in one loop)
    tokio::spawn(async move {
        loop {
            // Process ROS callbacks
            node.spin_once(std::time::Duration::from_millis(10));
            
            // Non-blocking check for new messages
            // r2r's next() is async, so we use a very short timeout or just a small sleep
            match tokio::time::timeout(std::time::Duration::from_millis(5), sub.next()).await {
                Ok(Some(msg)) => {
                    let x = msg.pose.pose.position.x;
                    let y = msg.pose.pose.position.y;
                    
                    let q = msg.pose.pose.orientation;
                    let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
                    let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
                    let theta = f64::atan2(siny_cosp, cosy_cosp);

                    let _ = tx.send(Telemetry { x, y, theta });
                }
                _ => {}
            }
            tokio::time::sleep(std::time::Duration::from_millis(10)).await;
        }
    });

    // TUI Initialization
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut app = App::new(rx);
    let res = run_app(&mut terminal, &mut app).await;

    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen, DisableMouseCapture)?;
    terminal.show_cursor()?;

    if let Err(err) = res { println!("{:?}", err) }
    Ok(())
}

async fn run_app<B: ratatui::backend::Backend>(
    terminal: &mut Terminal<B>,
    app: &mut App,
) -> io::Result<()> {
    loop {
        terminal.draw(|f| {
            let size = f.size();
            
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
                        Constraint::Length(1), // Subtitle (Backronym)
                        Constraint::Length(2), // Spacer
                        Constraint::Length(2), // Description
                        Constraint::Length(2), // Spacer
                        Constraint::Length(1), // Version/Build
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
                
                let subtitle = Paragraph::new(Line::from(vec![
                    Span::styled("T", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
                    Span::raw("actical "),
                    Span::styled("R", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
                    Span::raw("emote "),
                    Span::styled("I", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
                    Span::raw("nterface for "),
                    Span::styled("D", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
                    Span::raw("etailed "),
                    Span::styled("E", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
                    Span::raw("xploration and "),
                    Span::styled("N", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
                    Span::raw("avigation of "),
                    Span::styled("T", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
                    Span::raw("ITAN"),
                ])).alignment(Alignment::Center);

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
                let description = Paragraph::new(description_text)
                    .alignment(Alignment::Center);

                let metadata = Paragraph::new(Line::from(vec![
                    Span::styled("v0.1.0", Style::default().fg(Color::Cyan)),
                    Span::raw(" | "),
                    Span::styled("Target: ROS2 Jazzy", Style::default().fg(Color::Green)),
                    Span::raw(" | "),
                    Span::styled("Linux", Style::default().fg(Color::White)),
                ]))
                .alignment(Alignment::Center);

                let hint = Paragraph::new("\n\nPress [ENTER] to continue | [q] to quit")
                    .style(Style::default().fg(Color::DarkGray).add_modifier(Modifier::BOLD))
                    .alignment(Alignment::Center);

                f.render_widget(title, chunks[1]);
                f.render_widget(subtitle, chunks[2]);
                f.render_widget(description, chunks[4]); 
                f.render_widget(metadata, chunks[6]); // Shifted for spacer
                f.render_widget(hint, chunks[7]);

            } else if app.screen == Screen::Splash {
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Length(3), // Space
                        Constraint::Length(3), // Title
                        Constraint::Min(0),    // Checks
                        Constraint::Length(3), // Progress Bar
                        Constraint::Length(3), // Status
                    ].as_ref())
                    .split(size);

                let title = Paragraph::new("TRIDENT CONTROL SYSTEM v0.1.0")
                    .style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD))
                    .alignment(Alignment::Center)
                    .block(Block::default().borders(Borders::ALL));
                
                let checks: Vec<ListItem> = app.startup_checks.iter()
                    .map(|s| {
                        let color = if s.contains("NOT FOUND") || s.contains("DISCONNECTED") || s.contains("CRITICAL") { Color::Red }
                                   else if s.contains("Warning") || s.contains("LOW") || s.contains("CAPPED") || s.contains("Throttling") { Color::Yellow }
                                   else if s.contains("CONNECTED") || s.contains("STABLE") || s.contains("OK") { Color::Green }
                                   else { Color::Gray };
                        ListItem::new(s.as_str()).style(Style::default().fg(color))
                    }).collect();
                
                let check_list = List::new(checks)
                    .block(Block::default().borders(Borders::ALL).title(" SYSTEM DIAGNOSTICS "));

                let elapsed = app.splash_start.elapsed().as_secs_f32();
                let progress = (elapsed / 3.0).min(1.0);
                let gauge = Gauge::default()
                    .block(Block::default().borders(Borders::ALL).title(" INITIALIZING "))
                    .gauge_style(Style::default().fg(Color::Cyan).bg(Color::Black).add_modifier(Modifier::ITALIC))
                    .ratio(progress as f64);

                let status = Paragraph::new("INITIALIZING ROBOT...")
                    .style(Style::default().fg(Color::DarkGray))
                    .alignment(Alignment::Center);

                f.render_widget(title, chunks[1]);
                f.render_widget(check_list, chunks[2]);
                f.render_widget(gauge, chunks[3]);
                f.render_widget(status, chunks[4]);

                if progress >= 1.0 {
                    app.screen = Screen::Main;
                }
            } else if app.screen == Screen::DeviceSelect {
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Percentage(30),
                        Constraint::Length(10),
                        Constraint::Min(0),
                    ].as_ref())
                    .split(size);

                let block = Block::default()
                    .borders(Borders::ALL)
                    .title(" DEVICE SELECTION ")
                    .border_style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD));

                let items = vec![
                    ListItem::new("  1. TITAN (Robot) - Controller Dashboard "),
                    ListItem::new("  2. LAPTOP (Remote) - Teleop & Monitoring "),
                ];

                let list = List::new(items)
                    .block(block)
                    .highlight_style(Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD))
                    .highlight_symbol(">> ");

                let mut state = ratatui::widgets::ListState::default();
                state.select(Some(app.selection_index));

                // Center the selection box
                let area = centered_rect(60, 30, size);
                f.render_stateful_widget(list, area, &mut state);

                let hint = Paragraph::new("Press Up/Down to choose, Enter to confirm | [q] to quit")
                    .alignment(Alignment::Center)
                    .style(Style::default().fg(Color::DarkGray));
                f.render_widget(hint, chunks[2]);

            } else if app.screen == Screen::MapNameInput {
                let area = centered_rect(60, 20, size);
                
                let block = Block::default()
                    .borders(Borders::ALL)
                    .title(" SAVE MAP AS... ")
                    .border_style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD));

                let inner_area = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Length(1),
                        Constraint::Length(1),
                        Constraint::Min(0),
                    ].as_ref())
                    .split(area);

                let input_text = format!(" Name: {}_", app.map_name_input);
                let input = Paragraph::new(input_text)
                    .style(Style::default().fg(Color::Yellow))
                    .block(block);

                let hint = Paragraph::new("Enter: Save  |  Esc: Cancel")
                    .alignment(Alignment::Center)
                    .style(Style::default().fg(Color::DarkGray));

                f.render_widget(input, area);
                f.render_widget(hint, inner_area[2]);

            } else if app.screen == Screen::TeleopConfig {
                let area = centered_rect(50, 35, size);
                let block = Block::default()
                    .borders(Borders::ALL)
                    .title(" TELEOP CONFIGURATION ")
                    .border_style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD));
                
                let inner = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Length(3), // Speed field
                        Constraint::Length(3), // Turn field
                        Constraint::Min(0),    // Hint
                    ].as_ref())
                    .margin(2)
                    .split(area);

                let speed_style = if app.teleop_field_index == 0 { Style::default().fg(Color::Yellow) } else { Style::default().fg(Color::Gray) };
                let turn_style = if app.teleop_field_index == 1 { Style::default().fg(Color::Yellow) } else { Style::default().fg(Color::Gray) };

                let speed_field = Paragraph::new(format!(" Speed: {}_", app.teleop_speed))
                    .style(speed_style)
                    .block(Block::default().borders(Borders::ALL).title(" [LINEAR SPEED] "));
                
                let turn_field = Paragraph::new(format!(" Turn: {}_", app.teleop_turn))
                    .style(turn_style)
                    .block(Block::default().borders(Borders::ALL).title(" [ANGULAR TURN] "));

                let hint = Paragraph::new("Tab/Enter: Switch Field | Esc: Cancel | Enter (on last field): Launch")
                    .alignment(Alignment::Center)
                    .style(Style::default().fg(Color::DarkGray));

                f.render_widget(block, area);
                f.render_widget(speed_field, inner[0]);
                f.render_widget(turn_field, inner[1]);
                f.render_widget(hint, inner[2]);

            } else if app.screen == Screen::MapSelect {
                let area = centered_rect(40, 50, size);
                let block = Block::default()
                    .borders(Borders::ALL)
                    .title(" SELECT MAP FILE ")
                    .border_style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD));
                
                let items: Vec<ListItem> = app.available_maps
                    .iter()
                    .enumerate()
                    .map(|(i, m)| {
                        let style = if i == app.map_selection_index {
                            Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)
                        } else {
                            Style::default().fg(Color::White)
                        };
                        ListItem::new(format!(" {} ", m)).style(style)
                    })
                    .collect();

                let list = List::new(items)
                    .block(Block::default().borders(Borders::NONE))
                    .highlight_symbol(">> ");

                let hint = Paragraph::new("Arrows: Select | Enter: Launch | Esc: Cancel")
                    .alignment(Alignment::Center)
                    .style(Style::default().fg(Color::DarkGray));

                let inner = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Min(0),
                        Constraint::Length(1),
                    ].as_ref())
                    .margin(2)
                    .split(area);

                f.render_widget(block, area);
                f.render_widget(list, inner[0]);
                f.render_widget(hint, inner[1]);

            } else if app.screen == Screen::RVizConfigSelect {
                let area = centered_rect(40, 50, size);
                let block = Block::default()
                    .borders(Borders::ALL)
                    .title(" SELECT RVIZ CONFIG ")
                    .border_style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD));
                
                let items: Vec<ListItem> = app.available_rviz_configs
                    .iter()
                    .enumerate()
                    .map(|(i, m)| {
                        let style = if i == app.rviz_config_selection_index {
                            Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)
                        } else {
                            Style::default().fg(Color::White)
                        };
                        ListItem::new(format!(" {} ", m)).style(style)
                    })
                    .collect();

                let list = List::new(items)
                    .block(Block::default().borders(Borders::NONE))
                    .highlight_symbol(">> ");

                let hint = Paragraph::new("Arrows: Select | Enter: Launch | Esc: Cancel")
                    .alignment(Alignment::Center)
                    .style(Style::default().fg(Color::DarkGray));

                let inner = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Min(0),
                        Constraint::Length(1),
                    ].as_ref())
                    .margin(2)
                    .split(area);

                f.render_widget(block, area);
                f.render_widget(list, inner[0]);
                f.render_widget(hint, inner[1]);

            } else {
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Length(3), // Header
                        Constraint::Min(0),    // Main Body
                        Constraint::Length(3), // Footer
                    ].as_ref())
                    .split(size);

                let body_chunks = Layout::default()
                    .direction(Direction::Horizontal)
                    .constraints([
                        Constraint::Percentage(33), // Menu
                        Constraint::Percentage(27), // Odom
                        Constraint::Percentage(40), // Logs
                    ].as_ref())
                    .split(chunks[1]);

                // 1. Header with dynamic status
                let spinner = if app.active_process.is_some() {
                    let frames = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧"];
                    frames[app.spinner_frame]
                } else {
                    " "
                };

                let status_color = match app.operation_status.as_str() {
                    "MAPPING" => Color::Magenta,
                    "NAVIGATION" => Color::Green,
                    "BRINGUP" => Color::Blue,
                    _ => Color::DarkGray,
                };

                let header_text = vec![
                    Span::styled(format!(" {}  ", spinner), Style::default().fg(Color::Cyan)),
                    Span::styled("TRIDENT v0.2.0 ", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                    Span::styled(format!(" [ {} ] ", app.operation_status), Style::default().fg(status_color).add_modifier(Modifier::BOLD)),
                    Span::styled(format!("  {} ", spinner), Style::default().fg(Color::Cyan)),
                ];

                let header = Paragraph::new(Line::from(header_text))
                    .alignment(Alignment::Center)
                    .block(Block::default()
                        .borders(Borders::ALL)
                        .border_type(BorderType::Thick)
                        .border_style(Style::default().fg(Color::Cyan)));
                
                // 2. Menu
                let filtered_menu = app.get_filtered_menu();
                let menu_items: Vec<ListItem> = filtered_menu.iter().enumerate()
                    .map(|(i, item)| {
                        let style = if i == app.active_menu_index {
                            Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD).bg(Color::Rgb(40, 40, 40))
                        } else {
                            Style::default().fg(Color::White)
                        };
                        ListItem::new(format!("  {}", item.to_string())).style(style)
                    }).collect();
                
                let menu_list = List::new(menu_items)
                    .block(Block::default()
                        .borders(Borders::ALL)
                        .border_type(BorderType::Plain)
                        .title(" OPERATIONS ")
                        .border_style(Style::default().fg(Color::DarkGray)));

                // 3. Telemetry
                let t = &app.current_telemetry;
                let odom_text = vec![
                    Line::from(vec![
                        Span::styled(" [POS_X] ", Style::default().fg(Color::Cyan)),
                        Span::styled(format!("{:.3} m", t.x), Style::default().fg(Color::Yellow)),
                    ]),
                    Line::from(vec![
                        Span::styled(" [POS_Y] ", Style::default().fg(Color::Cyan)),
                        Span::styled(format!("{:.3} m", t.y), Style::default().fg(Color::Yellow)),
                    ]),
                    Line::from(vec![
                        Span::styled(" [THETA] ", Style::default().fg(Color::Cyan)),
                        Span::styled(format!("{:.3} rad", t.theta), Style::default().fg(Color::Yellow)),
                    ]),
                ];
                let odom_panel = Paragraph::new(odom_text)
                    .block(Block::default()
                        .borders(Borders::ALL)
                        .border_type(BorderType::Plain)
                        .title(" TELEMETRY ")
                        .border_style(Style::default().fg(Color::DarkGray)));

                // 4. Logs
                let display_logs = if app.logs.len() > 15 {
                    &app.logs[app.logs.len() - 15..]
                } else {
                    &app.logs[..]
                };
                let logs: Vec<ListItem> = display_logs.iter()
                    .map(|s| {
                        let color = if s.contains("Error") || s.contains("Failed") || s.contains("DISCONNECTED") || s.contains("CRITICAL") { Color::Red } 
                                   else if s.contains("RUN ON") { Color::Cyan }
                                   else if s.contains("Warning") || s.contains("LOW") || s.contains("CAPPED") || s.contains("Throttling") { Color::Yellow }
                                   else if s.contains("launched") || s.contains("complete") || s.contains("CONNECTED") || s.contains("STABLE") || s.contains("OK") { Color::Green }
                                   else { Color::Gray };
                        ListItem::new(format!("> {}", s)).style(Style::default().fg(color))
                    }).collect();
                
                let logs_panel = List::new(logs)
                    .block(Block::default()
                        .borders(Borders::ALL)
                        .border_type(BorderType::Plain)
                        .title(" SYSTEM LOGS ")
                        .border_style(Style::default().fg(Color::DarkGray)));

                // 5. Footer
                let footer_text = Line::from(vec![
                    Span::styled(" ROLE: ", Style::default().fg(Color::DarkGray)),
                    Span::styled(app.device_type.to_string(), Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                    Span::styled("  |  ", Style::default().fg(Color::DarkGray)),
                    Span::styled("\u{2191}\u{2193}", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
                    Span::styled(" SELECT  ", Style::default().fg(Color::White)),
                    Span::styled("|  ", Style::default().fg(Color::DarkGray)),
                    Span::styled("ENTER", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
                    Span::styled(" EXECUTE  ", Style::default().fg(Color::White)),
                    Span::styled("|  ", Style::default().fg(Color::DarkGray)),
                    Span::styled("Q", Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)),
                    Span::styled(" QUIT  ", Style::default().fg(Color::White)),
                    Span::styled("|  ", Style::default().fg(Color::DarkGray)),
                    Span::styled("C", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                    Span::styled(" CLEAR LOGS ", Style::default().fg(Color::White)),
                ]);

                let footer = Paragraph::new(footer_text)
                    .alignment(Alignment::Center)
                    .style(Style::default().fg(Color::DarkGray))
                    .block(Block::default()
                        .borders(Borders::ALL)
                        .border_type(BorderType::Plain)
                        .border_style(Style::default().fg(Color::DarkGray)));

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
                    match key.code {
                        KeyCode::Enter => app.screen = Screen::DeviceSelect,
                        KeyCode::Char('q') | KeyCode::Char('Q') => return Ok(()),
                        _ => {}
                    }
                } else if app.screen == Screen::Splash {
                    app.screen = Screen::DeviceSelect; // Skip splash on any key
                } else if app.screen == Screen::DeviceSelect {
                    match key.code {
                        KeyCode::Up => if app.selection_index > 0 { app.selection_index -= 1; },
                        KeyCode::Down => if app.selection_index < 1 { app.selection_index += 1; },
                        KeyCode::Enter => {
                            app.device_type = if app.selection_index == 0 { DeviceType::Titan } else { DeviceType::Laptop };
                            
                            // Initialize diagnostics and splash after selection
                            app.startup_checks.clear();
                            app.run_diagnostics();
                            app.splash_start = std::time::Instant::now();
                            app.screen = Screen::Splash;
                        },
                        KeyCode::Char('q') | KeyCode::Char('Q') => return Ok(()),
                        _ => {}
                    }
                } else if app.screen == Screen::MapNameInput {
                    match key.code {
                        KeyCode::Esc => app.screen = Screen::Main,
                        KeyCode::Backspace => { app.map_name_input.pop(); },
                        KeyCode::Char(c) => { app.map_name_input.push(c); },
                        KeyCode::Enter => {
                            if !app.map_name_input.trim().is_empty() {
                                let name = app.map_name_input.trim().to_string();
                                app.logs.push(format!("Saving map as: {}...", name));
                                
                                let path = format!("/home/pidev/titan_ws/src/titan_bringup/maps/{}", name);
                                let output = Command::new("ros2")
                                    .args([
                                        "run", "nav2_map_server", "map_saver_cli", 
                                        "-f", &path
                                    ])
                                    .output();
                                app.handle_output(output, &format!("Map '{}' saved.", name));
                                app.screen = Screen::Main;
                            }
                        },
                        _ => {}
                    }
                } else if app.screen == Screen::TeleopConfig {
                    if key.kind == crossterm::event::KeyEventKind::Press {
                        match key.code {
                            KeyCode::Esc => app.screen = Screen::Main,
                            KeyCode::Tab | KeyCode::Down => {
                                app.teleop_field_index = (app.teleop_field_index + 1) % 2;
                            },
                            KeyCode::Up => {
                                app.teleop_field_index = if app.teleop_field_index == 0 { 1 } else { 0 };
                            },
                            KeyCode::Backspace => {
                                if app.teleop_field_index == 0 { app.teleop_speed.pop(); }
                                else { app.teleop_turn.pop(); }
                            },
                            KeyCode::Delete => {
                                if app.teleop_field_index == 0 { app.teleop_speed.clear(); }
                                else { app.teleop_turn.clear(); }
                            },
                            KeyCode::Char(c) => {
                                if c.is_digit(10) || c == '.' || c == '-' {
                                    if app.teleop_field_index == 0 { app.teleop_speed.push(c); }
                                    else { app.teleop_turn.push(c); }
                                }
                            },
                            KeyCode::Enter => {
                                if app.teleop_field_index == 0 {
                                    app.teleop_field_index = 1;
                                } else {
                                    app.execute_selected();
                                }
                            },
                            _ => {}
                        }
                    }
                } else if app.screen == Screen::MapSelect {
                    if key.kind == crossterm::event::KeyEventKind::Press {
                        match key.code {
                            KeyCode::Esc => app.screen = Screen::Main,
                            KeyCode::Up => {
                                if app.map_selection_index > 0 { app.map_selection_index -= 1; }
                            },
                            KeyCode::Down => {
                                if app.map_selection_index < app.available_maps.len().saturating_sub(1) {
                                    app.map_selection_index += 1;
                                }
                            },
                            KeyCode::Enter => {
                                app.execute_selected();
                            },
                            _ => {}
                        }
                    }
                } else if app.screen == Screen::RVizConfigSelect {
                    if key.kind == crossterm::event::KeyEventKind::Press {
                        match key.code {
                            KeyCode::Esc => app.screen = Screen::Main,
                            KeyCode::Up => {
                                if app.rviz_config_selection_index > 0 { app.rviz_config_selection_index -= 1; }
                            },
                            KeyCode::Down => {
                                if app.rviz_config_selection_index < app.available_rviz_configs.len().saturating_sub(1) {
                                    app.rviz_config_selection_index += 1;
                                }
                            },
                            KeyCode::Enter => {
                                app.execute_selected();
                            },
                            _ => {}
                        }
                    }
                } else {
                    match key.code {
                        KeyCode::Char('q') | KeyCode::Char('Q') => return Ok(()),
                        KeyCode::Char('c') | KeyCode::Char('C') => app.logs.clear(),
                        KeyCode::Up => if app.active_menu_index > 0 { app.active_menu_index -= 1; },
                        KeyCode::Down => {
                            let max = app.get_filtered_menu().len().saturating_sub(1);
                            if app.active_menu_index < max { app.active_menu_index += 1; }
                        },
                        KeyCode::Enter => app.execute_selected(),
                        _ => {}
                    }
                }
            }
        }
        app.on_tick();
    }
}

fn centered_rect(percent_x: u16, percent_y: u16, r: Rect) -> Rect {
    let popup_layout = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Percentage((100 - percent_y) / 2),
            Constraint::Percentage(percent_y),
            Constraint::Percentage((100 - percent_y) / 2),
        ].as_ref())
        .split(r);

    Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage((100 - percent_x) / 2),
            Constraint::Percentage(percent_x),
            Constraint::Percentage((100 - percent_x) / 2),
        ].as_ref())
        .split(popup_layout[1])[1]
}
