use core::cell::LazyCell;
use core::sync::atomic::{AtomicBool, Ordering};

extern crate alloc;
use alloc::boxed::Box;

use common_messages::DroneResponse;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::pipe::Pipe;
use rtt_target::{UpChannel, rtt_init};

#[defmt::global_logger]
struct Logger;

struct Encoder {
    rtt_channel: LazyCell<UpChannel>,
    defmt_encoder: defmt::Encoder,
}

impl Encoder {
    const fn new() -> Self {
        fn init_up_channel() -> UpChannel {
            rtt_init! {
                up: {
                    0: {
                        size: 1024,
                        name: "defmt",
                    }
                }
            }
            .up
            .0
        }

        Self {
            rtt_channel: LazyCell::new(init_up_channel),
            defmt_encoder: defmt::Encoder::new(),
        }
    }

    fn start_frame(&mut self) {
        self.defmt_encoder.start_frame(|bytes| {
            self.rtt_channel.write(bytes);
            DEFMT_DATA.try_write(bytes).unwrap();
        });
    }

    fn end_frame(&mut self) {
        self.defmt_encoder.end_frame(|bytes| {
            self.rtt_channel.write(bytes);
            DEFMT_DATA.try_write(bytes).unwrap();
        });
    }

    fn write(&mut self, data: &[u8]) {
        self.defmt_encoder.write(data, |bytes| {
            self.rtt_channel.write(bytes);
            DEFMT_DATA.try_write(bytes).unwrap();
        });
    }
}

#[embassy_executor::task]
pub async fn defmt_data_to_drone_responses(
    drone_res: Sender<'static, CriticalSectionRawMutex, DroneResponse, 64>,
) {
    let mut buffer = [0; 1024];
    loop {
        let len = DEFMT_DATA.read(&mut buffer).await;
        drone_res
            .send(DroneResponse::Log(Box::from(&buffer[..len])))
            .await;
        embassy_futures::yield_now().await;
    }
}

static DEFMT_DATA: Pipe<CriticalSectionRawMutex, 1024> = Pipe::new();
static TAKEN: AtomicBool = AtomicBool::new(false);
static mut CS_RESTORE: critical_section::RestoreState = critical_section::RestoreState::invalid();
static mut ENCODER: Encoder = Encoder::new();

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        // safety: Must be paired with corresponding call to release(), see below
        let restore = unsafe { critical_section::acquire() };

        if TAKEN.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly")
        }

        // no need for CAS because interrupts are disabled
        TAKEN.store(true, Ordering::Relaxed);

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        unsafe { CS_RESTORE = restore };

        // safety: accessing the `static mut` is OK because we have disabled interrupts.
        unsafe {
            let encoder = &mut *core::ptr::addr_of_mut!(ENCODER);
            encoder.start_frame()
        }
    }

    unsafe fn flush() {}

    unsafe fn release() {
        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        let encoder = unsafe { &mut *core::ptr::addr_of_mut!(ENCODER) };
        encoder.end_frame();

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        TAKEN.store(false, Ordering::Relaxed);

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        let restore = unsafe { CS_RESTORE };

        // safety: Must be paired with corresponding call to acquire(), see above
        unsafe { critical_section::release(restore) };
    }

    unsafe fn write(bytes: &[u8]) {
        // safety: accessing the `static mut` is OK because we have disabled interrupts.
        let encoder = unsafe { &mut *core::ptr::addr_of_mut!(ENCODER) };
        encoder.write(bytes);
    }
}
