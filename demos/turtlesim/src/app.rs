use egui::Color32;

const DEFAULT_BG_R: u8 = 0x45;
const DEFAULT_BG_G: u8 = 0x56;
const DEFAULT_BG_B: u8 = 0xff;

/// We derive Deserialize/Serialize so we can persist app state on shutdown.
#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)] // if we add new fields, give them default values when deserializing old state
pub struct TurtlesimApp {}

impl Default for TurtlesimApp {
    fn default() -> Self {
        Self {}
    }
}

impl TurtlesimApp {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // This is also where you can customize the look and feel of egui using
        // `cc.egui_ctx.set_visuals` and `cc.egui_ctx.set_fonts`.
        // cc.egui_ctx.
        Default::default()
    }
}

impl eframe::App for TurtlesimApp {
    /// Called each time the UI needs repainting, which may be many times per second.
    /// Put your widgets into a `SidePanel`, `TopPanel`, `CentralPanel`, `Window` or `Area`.
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Examples of how to create different panels and windows.
        // Pick whichever suits you.
        // Tip: a good default choice is to just keep the `CentralPanel`.
        // For inspiration and more examples, go to https://emilk.github.io/egui

        let frame = egui::containers::Frame::default().fill(Color32::from_rgb(
            DEFAULT_BG_R,
            DEFAULT_BG_G,
            DEFAULT_BG_B,
        ));
        egui::CentralPanel::default().frame(frame).show(ctx, |ui| {
            egui::warn_if_debug_build(ui);
        });
    }
}
