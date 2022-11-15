#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]

pub use async_nrf52832_hal as hal;
use defmt_rtt as _; // global logger // memory layout
use panic_probe as _;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
