use core::cell::LazyCell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::zerocopy_channel;
use rtt_target::{UpChannel, rtt_init};

#[defmt::global_logger]
struct Logger;

struct Encoder {
    rtt_channel: LazyCell<UpChannel>,
    drone_res: zerocopy_channel::Sender,
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
            // TODO Send log frame
        });
    }

    fn end_frame(&mut self) {
        self.defmt_encoder.end_frame(|bytes| {
            self.rtt_channel.write(bytes);
            // TODO Send log frame
        });
    }

    fn write(&mut self, data: &[u8]) {
        self.defmt_encoder.write(data, |bytes| {
            self.rtt_channel.write(bytes);
            // TODO Send log frame
        });
    }
}

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
