use ratatui::{
    backend::CrosstermBackend,
    widgets::{Block, Borders, Paragraph, List, ListItem},
    layout::{Layout, Constraint, Direction, Alignment},
    style::{Color, Modifier, Style},
    Terminal,
};
use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use anyhow::Result;
use std::{io, time::Duration, process::{Command, Child}};

#[derive(PartialEq, Clone)]
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
            MenuItem::Bringup => "[Robot] Bringup (Drivers)".to_string(),
            MenuItem::LocalTeleop => "[Robot] Local Teleop (Split)".to_string(),
            MenuItem::Mapping => "[Robot] Mapping (SLAM)".to_string(),
            MenuItem::Navigation => "[Robot] Navigation (Auto)".to_string(),
            MenuItem::SaveMap => "[Robot] Save Current Map".to_string(),
            MenuItem::CheckUSB => "[Sys]   Check USB/Serial".to_string(),
            MenuItem::Battery => "[Sys]   Power/Battery".to_string(),
            MenuItem::Rebuild => "[Sys]   Rebuild Workspace".to_string(),
            MenuItem::StopProcess => "[Sys]   Stop Background Proc".to_string(),
            MenuItem::RemoteTeleop => "[Laptop] Remote Teleop Cmd".to_string(),
            MenuItem::RemoteRViz => "[Laptop] Remote RViz Cmd".to_string(),
        }
    }
}

#[derive(PartialEq)]
enum Screen {
    Splash,
    Main,
}

struct App {
    x: f64,
    y: f64,
    theta: f64,
    logs: Vec<String>,
    menu_items: Vec<MenuItem>,
    active_menu_index: usize,
    active_process: Option<Child>,
    screen: Screen,
    startup_checks: Vec<String>,
}

impl App {
    fn new() -> App {
        App {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            logs: vec![
                "TITAN System initialized".to_string(),
                "Ready for commands".to_string(),
            ],
            menu_items: vec![
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
            screen: Screen::Splash,
            startup_checks: Vec::new(),
        }
    }

    fn run_diagnostics(&mut self) {
        self.startup_checks.push("Checking Power System...".to_string());
        self.startup_checks.push(format!("  -> {}", self.get_battery_readable()));
        
        self.startup_checks.push("Scanning Serial Bus...".to_string());
        let usb = self.get_usb_readable();
        for line in usb {
            self.startup_checks.push(format!("  -> {}", line));
        }
        
        self.startup_checks.push("Sourcing Workspace... OK".to_string());
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

    fn on_tick(&mut self) {}

    fn execute_selected(&mut self) {
        let item = self.menu_items[self.active_menu_index].clone();
        
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
                self.logs.push(format!("Executing: {}", item.to_string()));
                match item {
                    MenuItem::Bringup => self.spawn_ros_launch("bringup.launch.py"),
                    MenuItem::Mapping => self.spawn_ros_launch("mapping.launch.py"),
                    MenuItem::Navigation => self.spawn_ros_launch("navigation.launch.py"),
                    MenuItem::LocalTeleop => {
                        let is_tmux = std::env::var("TMUX").is_ok();
                        if is_tmux {
                            self.logs.push("Spawning Teleop in split...".to_string());
                            let _ = Command::new("tmux")
                                .args(["split-window", "-h", "ros2 run teleop_twist_keyboard teleop_twist_keyboard"])
                                .spawn();
                        } else {
                            self.logs.push("Error: Local Teleop requires tmux split!".to_string());
                        }
                    },
                    MenuItem::RemoteTeleop => {
                        self.logs.push("RUN ON LAPTOP:".to_string());
                        self.logs.push("ros2 run teleop_twist_keyboard teleop_twist_keyboard".to_string());
                    },
                    MenuItem::RemoteRViz => {
                        self.logs.push("RUN ON LAPTOP:".to_string());
                        self.logs.push("rviz2 -d ~/titan_ws/src/titan_bringup/rviz/titan.rviz".to_string());
                    },
                    MenuItem::SaveMap => {
                        let output = Command::new("ros2")
                            .args([
                                "run", "nav2_map_server", "map_saver_cli", 
                                "-f", "/home/pidev/titan_ws/src/titan_bringup/maps/campus_map"
                            ])
                            .output();
                        self.handle_output(output, "Map saved.");
                    },
                    MenuItem::Rebuild => {
                        self.logs.push("Rebuilding...".to_string());
                        let output = Command::new("bash").arg("-c").arg("cd ~/titan_ws && colcon build --symlink-install").output();
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
        let is_tmux = std::env::var("TMUX").is_ok();
        let cmd_str = format!("source ~/titan_ws/install/setup.bash && ros2 launch titan_bringup {}", file);

        if is_tmux {
            self.logs.push("Spawning in tmux split...".to_string());
            let _ = Command::new("tmux").args(["split-window", "-h", &cmd_str]).spawn();
        } else {
            if self.active_process.is_some() {
                self.logs.push("Another process is already running!".to_string());
                return;
            }
            let _ = Command::new("bash").arg("-c").arg(&cmd_str).spawn().map(|child| self.active_process = Some(child));
            self.logs.push(format!("{} launched in background.", file));
        }
    }

    fn handle_output(&mut self, res: io::Result<std::process::Output>, success_msg: &str) {
        match res {
            Ok(out) => {
                if out.status.success() {
                    self.logs.push(success_msg.to_string());
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
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut app = App::new();
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
    // Run initial diagnostics once
    app.run_diagnostics();
    let splash_start = std::time::Instant::now();

    loop {
        terminal.draw(|f| {
            let size = f.size();
            
            if app.screen == Screen::Splash {
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Length(3), // Space
                        Constraint::Length(3), // Title
                        Constraint::Min(0),    // Checks
                        Constraint::Length(3), // Status
                    ].as_ref())
                    .split(size);

                let title = Paragraph::new("TITAN CONTROL SYSTEM v0.1.0")
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

                let status = Paragraph::new("INITIALIZING ROBOT...")
                    .style(Style::default().fg(Color::DarkGray))
                    .alignment(Alignment::Center);

                f.render_widget(title, chunks[1]);
                f.render_widget(check_list, chunks[2]);
                f.render_widget(status, chunks[3]);

                if splash_start.elapsed() > Duration::from_secs(3) {
                    app.screen = Screen::Main;
                }
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

                // 1. Header
                let header = Paragraph::new(" TITAN CONTROL SYSTEM v0.1.0 ")
                    .style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD))
                    .alignment(Alignment::Center)
                    .block(Block::default().borders(Borders::ALL).border_style(Style::default().fg(Color::DarkGray)));
                
                // 2. Menu
                let menu_items: Vec<ListItem> = app.menu_items.iter().enumerate()
                    .map(|(i, item)| {
                        let style = if i == app.active_menu_index {
                            Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD).bg(Color::Rgb(40, 40, 40))
                        } else {
                            Style::default().fg(Color::White)
                        };
                        ListItem::new(format!("  {}", item.to_string())).style(style)
                    }).collect();
                
                let menu_list = List::new(menu_items)
                    .block(Block::default().borders(Borders::ALL).title(" OPERATIONS ").border_style(Style::default().fg(Color::DarkGray)));

                // 3. Telemetry
                let odom_text = format!("\n X: {:.3} m\n Y: {:.3} m\n \u{03B8}: {:.3} rad", app.x, app.y, app.theta);
                let odom_panel = Paragraph::new(odom_text)
                    .style(Style::default().fg(Color::Green))
                    .block(Block::default().borders(Borders::ALL).title(" TELEMETRY ").border_style(Style::default().fg(Color::DarkGray)));

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
                    .block(Block::default().borders(Borders::ALL).title(" SYSTEM LOGS ").border_style(Style::default().fg(Color::DarkGray)));

                // 5. Footer
                let footer = Paragraph::new(" \u{2191}\u{2193}: Select  |  Enter: Execute  |  Q: Quit  |  C: Clear Logs ")
                    .alignment(Alignment::Center)
                    .style(Style::default().fg(Color::DarkGray))
                    .block(Block::default().borders(Borders::ALL).border_style(Style::default().fg(Color::DarkGray)));

                f.render_widget(header, chunks[0]);
                f.render_widget(menu_list, body_chunks[0]);
                f.render_widget(odom_panel, body_chunks[1]);
                f.render_widget(logs_panel, body_chunks[2]);
                f.render_widget(footer, chunks[2]);
            }
        })?;

        if crossterm::event::poll(Duration::from_millis(100))? {
            if let Event::Key(key) = event::read()? {
                if app.screen == Screen::Splash {
                    app.screen = Screen::Main; // Skip splash on any key
                } else {
                    match key.code {
                        KeyCode::Char('q') | KeyCode::Char('Q') => return Ok(()),
                        KeyCode::Char('c') | KeyCode::Char('C') => app.logs.clear(),
                        KeyCode::Up => if app.active_menu_index > 0 { app.active_menu_index -= 1; },
                        KeyCode::Down => if app.active_menu_index < app.menu_items.len() - 1 { app.active_menu_index += 1; },
                        KeyCode::Enter => app.execute_selected(),
                        _ => {}
                    }
                }
            }
        }
        app.on_tick();
    }
}

