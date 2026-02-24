use static_cell::ConstStaticCell;

pub mod bmi323;
pub mod lsm6ds3;

const SPI_BUF_LEN: usize = 8192;
static SPI_BUF: ConstStaticCell<[u8; SPI_BUF_LEN]> = ConstStaticCell::new([0u8; SPI_BUF_LEN]);
