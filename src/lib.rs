#![no_std]
#![feature(async_fn_in_trait)]

pub use nrf52832_hal;
pub use nrf52832_hal::pac;

pub use embedded_hal;
pub use embedded_hal_async;

pub mod gpio;
pub mod gpiote;
pub mod interrupt_token;
pub mod spim;
pub mod waker_registration;

pub(crate) struct DmaSlice {
    ptr: u32,
    len: u32,
}

impl DmaSlice {
    pub fn null() -> Self {
        Self { ptr: 0, len: 0 }
    }

    pub fn from_slice(slice: &[u8]) -> Self {
        Self {
            ptr: slice.as_ptr() as u32,
            len: slice.len() as u32,
        }
    }
}

/// Does this slice reside entirely within RAM?
pub(crate) fn slice_in_ram(slice: &[u8]) -> bool {
    let ptr = slice.as_ptr() as usize;
    ptr >= nrf52832_hal::target_constants::SRAM_LOWER
        && (ptr + slice.len()) < nrf52832_hal::target_constants::SRAM_UPPER
}

/// Return an error if slice is not in RAM.
pub(crate) fn slice_in_ram_or<T>(slice: &[u8], err: T) -> Result<(), T> {
    if slice_in_ram(slice) {
        Ok(())
    } else {
        Err(err)
    }
}
