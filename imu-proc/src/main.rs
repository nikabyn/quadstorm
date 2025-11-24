#![feature(file_buffered, trim_prefix_suffix)]

use std::{collections::VecDeque, io::BufRead};

use base64::Engine;

#[derive(Debug, Clone, Copy)]
enum SampleEvent {
    Ok(Sample),
    Lagged(Sample),
}

#[derive(Debug, Clone, Copy, Default)]
struct Sample {
    idx: u64,
    gy: [f32; 3],
    xl: [f32; 3],
    temp: [f32; 3],
}

impl Sample {
    fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(bytes.len(), 8 + 3 * 3 * 4);

        let (u64_chunks, _) = bytes.as_chunks::<8>();
        let idx = u64::from_le_bytes(*u64_chunks.first().unwrap());

        let bytes = &bytes[8..];
        let (le_floats, _) = bytes.as_chunks::<4>();
        let mut floats = le_floats.iter().map(|&b| f32::from_le_bytes(b));

        Sample {
            idx,
            gy: [
                floats.next().unwrap(),
                floats.next().unwrap(),
                floats.next().unwrap(),
            ],
            xl: [
                floats.next().unwrap(),
                floats.next().unwrap(),
                floats.next().unwrap(),
            ],
            temp: [
                floats.next().unwrap(),
                floats.next().unwrap(),
                floats.next().unwrap(),
            ],
        }
    }
}

fn main() -> eyre::Result<()> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or("/dev/ttyACM0".to_string());
    let (ctx_tx, ctx_rx) = std::sync::mpsc::sync_channel(1);
    let (data_pump, sample_rx) = data_pump(path, ctx_rx);

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([350.0, 200.0]),
        ..Default::default()
    };

    eframe::run_native(
        "My egui App with a plot",
        options,
        Box::new(move |cc| {
            ctx_tx.send(cc.egui_ctx.clone()).unwrap();

            Ok(Box::new(ImuVis {
                sample_rx,
                gy: Default::default(),
                xl: Default::default(),
                temp: Default::default(),
            }))
        }),
    )
    .unwrap();

    data_pump.join().unwrap();
    Ok(())
}

fn data_pump(
    path: String,
    egui_ctx_rx: std::sync::mpsc::Receiver<egui::Context>,
) -> (
    std::thread::JoinHandle<()>,
    std::sync::mpsc::Receiver<SampleEvent>,
) {
    let (tx, rx) = std::sync::mpsc::sync_channel(64);

    let handle = std::thread::spawn(move || {
        let egui_ctx = egui_ctx_rx.recv().unwrap();
        drop(egui_ctx_rx);

        let mut stream = std::fs::File::open_buffered(path).unwrap().lines();
        while let Some(Ok(line)) = stream.next() {
            println!("[esp32] {line}");

            if let Some(Ok(sample_bytes)) = line.split_once("B64:").map(|(_, b64)| {
                base64::prelude::BASE64_STANDARD_NO_PAD.decode(b64.trim_suffix("\u{1b}[0m"))
            }) && sample_bytes.len() == 45
            {
                let tag = sample_bytes[0];
                let sample = Sample::from_bytes(&sample_bytes[1..45]);

                let event = match tag {
                    b'O' => SampleEvent::Ok(sample),
                    b'L' => SampleEvent::Lagged(sample),
                    _ => unreachable!(),
                };

                if tx.send(event).is_err() {
                    return;
                }
                egui_ctx.request_repaint();
            }
        }

        println!("[!] data EOF");
    });

    (handle, rx)
}

struct ImuVis {
    sample_rx: std::sync::mpsc::Receiver<SampleEvent>,

    gy: [VecDeque<egui_plot::PlotPoint>; 3],
    xl: [VecDeque<egui_plot::PlotPoint>; 3],
    temp: [VecDeque<egui_plot::PlotPoint>; 1],
}

impl eframe::App for ImuVis {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        while let Ok(sample) = self.sample_rx.try_recv() {
            let sample = match sample {
                SampleEvent::Ok(sample) | SampleEvent::Lagged(sample) => sample,
            };

            const MAX_POINTS: usize = 1600 * 10;

            for i in 0..self.gy.len() {
                if self.gy[i].len() > MAX_POINTS {
                    _ = self.gy[i].pop_front();
                }

                self.gy[i].push_back(egui_plot::PlotPoint::new(sample.idx as f64, sample.gy[i]));
                self.gy[i].make_contiguous();
            }
            for i in 0..self.xl.len() {
                if self.xl[i].len() > MAX_POINTS {
                    _ = self.xl[i].pop_front();
                }

                self.xl[i].push_back(egui_plot::PlotPoint::new(sample.idx as f64, sample.xl[i]));
                self.xl[i].make_contiguous();
            }
            for i in 0..self.temp.len() {
                if self.temp[i].len() > MAX_POINTS {
                    _ = self.temp[i].pop_front();
                }

                self.temp[i]
                    .push_back(egui_plot::PlotPoint::new(sample.idx as f64, sample.temp[i]));
                self.temp[i].make_contiguous();
            }
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            egui::Grid::new("plot_grid")
                .num_columns(2)
                .min_row_height(420.0)
                .show(ui, |ui| self.draw_plots(ui));
        });
    }
}

impl ImuVis {
    fn draw_plots(&self, ui: &mut egui::Ui) {
        self.draw_plot(ui, "Gyro", &self.gy);
        self.draw_plot(ui, "Accelerometer", &self.xl);
        self.draw_plot(ui, "Temperature", &self.temp);
    }

    fn draw_plot(&self, ui: &mut egui::Ui, name: &str, data: &[VecDeque<egui_plot::PlotPoint>]) {
        ui.label(name);
        egui_plot::Plot::new(name.to_lowercase().replace(' ', "_"))
            .legend(egui_plot::Legend::default().position(egui_plot::Corner::LeftTop))
            .show(ui, |plot_ui| {
                let labels = ["x", "y", "z"].into_iter();
                for (label, data) in std::iter::zip(labels, data.iter()) {
                    plot_ui.line(egui_plot::Line::new(
                        label,
                        egui_plot::PlotPoints::Borrowed(data.as_slices().0),
                    ));
                }
            });
        ui.end_row();
    }
}
