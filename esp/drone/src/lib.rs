#![no_std]
pub mod esp_ikarus;

#[macro_export]
macro_rules! make_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: std::cell::OnceCell<$t> = std::cell::OnceCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
