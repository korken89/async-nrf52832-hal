#![no_std]
#![feature(async_fn_in_trait)]

// pub use nrf52832_hal;
// pub use nrf52832_hal::pac;
pub use nrf52832_pac as pac;

pub use embedded_hal;
pub use embedded_hal_async;

pub mod clocks;
pub mod gpio;
pub mod gpiote;
pub mod saadc;
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

/// This marker is implemented on an interupt token to enforce that the right tokens
/// are given to the correct HAL peripheral.
///
/// This trait is implemented by the HAL, not intended for user implementation.
pub unsafe trait InterruptToken<Periperhal> {}

/// Does this slice reside entirely within RAM?
pub(crate) fn slice_in_ram(slice: &[u8]) -> bool {
    let ptr = slice.as_ptr() as usize;
    ptr >= target_constants::SRAM_LOWER && (ptr + slice.len()) < target_constants::SRAM_UPPER
}

/// Return an error if slice is not in RAM.
pub(crate) fn slice_in_ram_or<T>(slice: &[u8], err: T) -> Result<(), T> {
    if slice_in_ram(slice) {
        Ok(())
    } else {
        Err(err)
    }
}

struct OnDrop<F: FnOnce()> {
    f: core::mem::MaybeUninit<F>,
}

impl<F: FnOnce()> OnDrop<F> {
    pub fn new(f: F) -> Self {
        Self {
            f: core::mem::MaybeUninit::new(f),
        }
    }

    pub fn defuse(self) {
        core::mem::forget(self)
    }
}

impl<F: FnOnce()> Drop for OnDrop<F> {
    fn drop(&mut self) {
        unsafe { self.f.as_ptr().read()() }
    }
}

/// Length of Nordic EasyDMA differs for MCUs
pub mod target_constants {
    #[cfg(feature = "51")]
    pub const EASY_DMA_SIZE: usize = (1 << 8) - 1;
    #[cfg(feature = "52805")]
    pub const EASY_DMA_SIZE: usize = (1 << 14) - 1;
    #[cfg(feature = "52810")]
    pub const EASY_DMA_SIZE: usize = (1 << 10) - 1;
    #[cfg(feature = "52811")]
    pub const EASY_DMA_SIZE: usize = (1 << 14) - 1;
    #[cfg(feature = "52820")]
    pub const EASY_DMA_SIZE: usize = (1 << 15) - 1;
    // #[cfg(feature = "52832")]
    pub const EASY_DMA_SIZE: usize = (1 << 8) - 1;
    #[cfg(feature = "52833")]
    pub const EASY_DMA_SIZE: usize = (1 << 16) - 1;
    #[cfg(feature = "52840")]
    pub const EASY_DMA_SIZE: usize = (1 << 16) - 1;
    #[cfg(any(feature = "5340-app", feature = "5340-net"))]
    pub const EASY_DMA_SIZE: usize = (1 << 16) - 1;
    #[cfg(feature = "9160")]
    pub const EASY_DMA_SIZE: usize = (1 << 12) - 1;

    // Limits for Easy DMA - it can only read from data ram
    pub const SRAM_LOWER: usize = 0x2000_0000;
    pub const SRAM_UPPER: usize = 0x3000_0000;

    // #[cfg(any(feature = "51", feature = "52810", feature = "52832"))]
    pub const FORCE_COPY_BUFFER_SIZE: usize = 255;
    // #[cfg(not(any(feature = "51", feature = "52810", feature = "52832")))]
    // pub const FORCE_COPY_BUFFER_SIZE: usize = 1024;
    const _CHECK_FORCE_COPY_BUFFER_SIZE: usize = EASY_DMA_SIZE - FORCE_COPY_BUFFER_SIZE;
    // ERROR: FORCE_COPY_BUFFER_SIZE must be <= EASY_DMA_SIZE
}
