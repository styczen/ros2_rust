#![warn(clippy::all, rust_2018_idioms)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

// #define DEFAULT_BG_R 0x45
// #define DEFAULT_BG_G 0x56
// #define DEFAULT_BG_B 0xff

fn main() -> eframe::Result<()> {
    env_logger::init(); // Log to stderr (if you run with `RUST_LOG=debug`).

    let mut native_options = eframe::NativeOptions::default();
    native_options.min_window_size = Some(egui::Vec2{x: 500.0, y: 500.0}); 
    native_options.max_window_size = Some(egui::Vec2{x: 500.0, y: 500.0}); 
    eframe::run_native(
        "Turtlesim",
        native_options,
        Box::new(|cc| Box::new(eframe_template::TemplateApp::new(cc))),
    )
}
