//! HAL interface to the SPIM peripheral.
//!
//! See product specification, chapter 31.

use crate::gpio::{Floating, Input, Output, Pin, PushPull};
use crate::target_constants::EASY_DMA_SIZE;
use crate::waker_registration::CriticalSectionWakerRegistration;
use crate::{slice_in_ram_or, DmaSlice, InterruptToken};
use core::ops::Deref;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::{Poll, Waker};
pub use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3};
use nrf52832_pac::{spim0, SPIM0, SPIM1, SPIM2};
pub use spim0::frequency::FREQUENCY_A as Frequency;

/// SPIM events
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum SpimEvent {
    End,
    EndRx,
    EndTx,
    Stopped,
    Started,
}

#[macro_export]
macro_rules! register_spim0_interrupt {
    ($token_name:ident) => {
        pub struct $token_name;

        unsafe impl async_nrf52832_hal::InterruptToken<async_nrf52832_hal::spim::export::SPIM0>
            for $token_name
        {
        }

        #[no_mangle]
        #[allow(non_snake_case)]
        unsafe extern "C" fn SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0() {
            async_nrf52832_hal::spim::export::on_interrupt_spim0();
        }
    };
}

#[macro_export]
macro_rules! register_spim1_interrupt {
    ($token_name:ident) => {
        pub struct $token_name;

        unsafe impl async_nrf52832_hal::InterruptToken<async_nrf52832_hal::spim::export::SPIM1>
            for $token_name
        {
        }

        #[no_mangle]
        #[allow(non_snake_case)]
        unsafe extern "C" fn SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1() {
            async_nrf52832_hal::spim::export::on_interrupt_spim1();
        }
    };
}

#[macro_export]
macro_rules! register_spim2_interrupt {
    ($token_name:ident) => {
        pub struct $token_name;

        unsafe impl async_nrf52832_hal::InterruptToken<async_nrf52832_hal::spim::export::SPIM2>
            for $token_name
        {
        }

        #[no_mangle]
        #[allow(non_snake_case)]
        unsafe extern "C" fn SPIM2_SPIS2_SPI2() {
            async_nrf52832_hal::spim::export::on_interrupt_spim2();
        }
    };
}

static SPIM0_WAKER: CriticalSectionWakerRegistration = CriticalSectionWakerRegistration::new();
static SPIM1_WAKER: CriticalSectionWakerRegistration = CriticalSectionWakerRegistration::new();
static SPIM2_WAKER: CriticalSectionWakerRegistration = CriticalSectionWakerRegistration::new();

// Hidden export only for use by the macro
#[doc(hidden)]
pub mod export {
    pub use nrf52832_pac::{SPIM0, SPIM1, SPIM2};

    /// On interrupt.
    pub fn on_interrupt_spim0() {
        let spim = unsafe { &*SPIM0::PTR };

        if spim.events_end.read().bits() != 0 {
            super::SPIM0_WAKER.wake();
            spim.intenclr.write(|w| w.end().clear());
        }
    }

    /// On interrupt.
    pub fn on_interrupt_spim1() {
        let spim = unsafe { &*SPIM1::PTR };

        if spim.events_end.read().bits() != 0 {
            super::SPIM1_WAKER.wake();
            spim.intenclr.write(|w| w.end().clear());
        }
    }

    /// On interrupt.
    pub fn on_interrupt_spim2() {
        let spim = unsafe { &*SPIM2::PTR };

        if spim.events_end.read().bits() != 0 {
            super::SPIM2_WAKER.wake();
            spim.intenclr.write(|w| w.end().clear());
        }
    }
}

/// Interface to a SPIM instance.
///
/// This is a very basic interface that comes with the following limitations:
/// - The SPIM instances share the same address space with instances of SPIS,
///   SPI, TWIM, TWIS, and TWI. You need to make sure that conflicting instances
///   are disabled before using `Spim`. See product specification, section 15.2.
pub struct Spim<T>(T)
where
    T: Instance;

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

impl<T> embedded_hal::spi::ErrorType for Spim<T>
where
    T: Instance,
{
    type Error = Error;
}

impl<T> embedded_hal::spi::SpiBus for Spim<T>
where
    T: Instance,
{
    fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
        unimplemented!()
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        slice_in_ram_or(words, Error::DMABufferNotInDataMemory)?;

        words.chunks(EASY_DMA_SIZE).try_for_each(|chunk| {
            self.do_spi_dma_transfer(DmaSlice::from_slice(chunk), DmaSlice::from_slice(chunk))
        })?;

        Ok(())
    }
}

impl<T> embedded_hal::spi::SpiBusWrite for Spim<T>
where
    T: Instance,
{
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        slice_in_ram_or(words, Error::DMABufferNotInDataMemory)?;

        words.chunks(EASY_DMA_SIZE).try_for_each(|chunk| {
            self.do_spi_dma_transfer(DmaSlice::from_slice(chunk), DmaSlice::null())
        })?;

        Ok(())
    }
}

impl<T> embedded_hal::spi::SpiBusRead for Spim<T>
where
    T: Instance,
{
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        embedded_hal::spi::SpiBus::transfer_in_place(self, words)
    }
}

impl<T> embedded_hal::spi::SpiBusFlush for Spim<T>
where
    T: Instance,
{
    fn flush(&mut self) -> Result<(), Self::Error> {
        // Noop
        Ok(())
    }
}

impl<T> Spim<T>
where
    T: Instance,
{
    /// Enables interrupt for specified event.
    #[inline(always)]
    pub fn enable_interrupt(&self, event: SpimEvent) -> &Self {
        self.0.intenset.modify(|_r, w| match event {
            SpimEvent::End => w.end().set_bit(),
            SpimEvent::EndRx => w.endrx().set_bit(),
            SpimEvent::EndTx => w.endtx().set_bit(),
            SpimEvent::Stopped => w.stopped().set_bit(),
            SpimEvent::Started => w.started().set_bit(),
        });
        self
    }

    /// Disables interrupt for specified event.
    #[inline(always)]
    pub fn disable_interrupt(&self, event: SpimEvent) -> &Self {
        self.0.intenclr.write(|w| match event {
            SpimEvent::End => w.end().set_bit(),
            SpimEvent::EndRx => w.endrx().set_bit(),
            SpimEvent::EndTx => w.endtx().set_bit(),
            SpimEvent::Stopped => w.stopped().set_bit(),
            SpimEvent::Started => w.started().set_bit(),
        });
        self
    }

    /// Resets specified event.
    #[inline(always)]
    pub fn reset_event(&self, event: SpimEvent) {
        match event {
            SpimEvent::End => self.0.events_end.reset(),
            SpimEvent::EndRx => self.0.events_endrx.reset(),
            SpimEvent::EndTx => self.0.events_endtx.reset(),
            SpimEvent::Stopped => self.0.events_stopped.reset(),
            SpimEvent::Started => self.0.events_started.reset(),
        };
    }

    /// Checks if specified event has been triggered.
    #[inline(always)]
    pub fn is_event_triggered(&self, event: SpimEvent) -> bool {
        match event {
            SpimEvent::End => self.0.events_end.read().bits() != 0,
            SpimEvent::EndRx => self.0.events_endrx.read().bits() != 0,
            SpimEvent::EndTx => self.0.events_endtx.read().bits() != 0,
            SpimEvent::Stopped => self.0.events_stopped.read().bits() != 0,
            SpimEvent::Started => self.0.events_started.read().bits() != 0,
        }
    }

    pub fn new(
        spim: T,
        pins: Pins,
        frequency: Frequency,
        mode: Mode,
        orc: u8,
        _interrupt_token: impl InterruptToken<T>,
    ) -> Self {
        // Select pins.
        spim.psel.sck.write(|w| {
            unsafe { w.bits(pins.sck.psel_bits()) };
            w.connect().connected()
        });

        match pins.mosi {
            Some(mosi) => spim.psel.mosi.write(|w| {
                unsafe { w.bits(mosi.psel_bits()) };
                w.connect().connected()
            }),
            None => spim.psel.mosi.write(|w| w.connect().disconnected()),
        }
        match pins.miso {
            Some(miso) => spim.psel.miso.write(|w| {
                unsafe { w.bits(miso.psel_bits()) };
                w.connect().connected()
            }),
            None => spim.psel.miso.write(|w| w.connect().disconnected()),
        }

        // Enable SPIM instance.
        spim.enable.write(|w| w.enable().enabled());

        // Configure mode.
        spim.config.write(|w| {
            // Can't match on `mode` due to embedded-hal, see https://github.com/rust-embedded/embedded-hal/pull/126
            if mode == MODE_0 {
                w.order().msb_first();
                w.cpol().active_high();
                w.cpha().leading();
            } else if mode == MODE_1 {
                w.order().msb_first();
                w.cpol().active_high();
                w.cpha().trailing();
            } else if mode == MODE_2 {
                w.order().msb_first();
                w.cpol().active_low();
                w.cpha().leading();
            } else {
                w.order().msb_first();
                w.cpol().active_low();
                w.cpha().trailing();
            }
            w
        });

        // Configure frequency.
        spim.frequency.write(|w| w.frequency().variant(frequency));

        // Set over-read character to `0`.
        spim.orc.write(|w|
            // The ORC field is 8 bits long, so `0` is a valid value to write
            // there.
            unsafe { w.orc().bits(orc) });

        T::unmask_interrupt();

        Spim(spim)
    }

    fn start_dma_transfer(&mut self, tx: &DmaSlice, rx: &DmaSlice) {
        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // before any DMA action has started.
        compiler_fence(Ordering::SeqCst);

        // Set up the DMA write.
        self.0.txd.ptr.write(|w| unsafe { w.ptr().bits(tx.ptr) });

        self.0.txd.maxcnt.write(|w|
            // Note that that nrf52840 maxcnt is a wider.
            // type than a u8, so we use a `_` cast rather than a `u8` cast.
            // The MAXCNT field is thus at least 8 bits wide and accepts the full
            // range of values that fit in a `u8`.
            unsafe { w.maxcnt().bits(tx.len as _ ) });

        // Set up the DMA read.
        self.0.rxd.ptr.write(|w|
            // This is safe for the same reasons that writing to TXD.PTR is
            // safe. Please refer to the explanation there.
            unsafe { w.ptr().bits(rx.ptr) });
        self.0.rxd.maxcnt.write(|w|
            // This is safe for the same reasons that writing to TXD.MAXCNT is
            // safe. Please refer to the explanation there.
            unsafe { w.maxcnt().bits(rx.len as _) });

        // Reset and enable the end event
        self.reset_done();
        self.enable_interrupt(SpimEvent::End);

        // Start SPI transaction.
        self.0.tasks_start.write(|w|
            // `1` is a valid value to write to task registers.
            unsafe { w.bits(1) });

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // after all possible DMA actions have completed.
        compiler_fence(Ordering::SeqCst);
    }

    /// Internal helper function to setup and execute SPIM DMA transfer.
    fn do_spi_dma_transfer(&mut self, tx: DmaSlice, rx: DmaSlice) -> Result<(), Error> {
        self.start_dma_transfer(&tx, &rx);

        // Wait for END event.
        //
        // This event is triggered once both transmitting and receiving are
        // done.
        while !self.is_done() {}

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // after all possible DMA actions have completed.
        compiler_fence(Ordering::SeqCst);

        if self.0.txd.amount.read().bits() != tx.len {
            return Err(Error::Transmit);
        }
        if self.0.rxd.amount.read().bits() != rx.len {
            return Err(Error::Receive);
        }
        Ok(())
    }

    /// Internal helper
    async fn async_do_spi_dma_transfer(&mut self, tx: DmaSlice, rx: DmaSlice) -> Result<(), Error> {
        let dropper = crate::OnDrop::new(|| {
            Self::stop();
        });

        self.start_dma_transfer(&tx, &rx);

        core::future::poll_fn(|cx| {
            T::register_waker(cx.waker());

            if self.is_done() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        dropper.defuse();

        if self.0.txd.amount.read().bits() != tx.len {
            return Err(Error::Transmit);
        }
        if self.0.rxd.amount.read().bits() != rx.len {
            return Err(Error::Receive);
        }
        Ok(())
    }

    /// SPI transfer where both buffers are in RAM. Data must be in RAM.
    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        self.async_do_spi_dma_transfer(DmaSlice::from_slice(read), DmaSlice::from_slice(write))
            .await
    }

    /// Simultaneously sends and receives data. Places the received data into the same buffer.
    async fn transfer_in_place(&mut self, data: &mut [u8]) -> Result<(), Error> {
        self.async_do_spi_dma_transfer(DmaSlice::from_slice(data), DmaSlice::from_slice(data))
            .await
    }

    /// Sends data, discarding any received data. Data must be in RAM.
    async fn write(&mut self, data: &[u8]) -> Result<(), Error> {
        self.async_do_spi_dma_transfer(DmaSlice::null(), DmaSlice::from_slice(data))
            .await
    }

    /// Reads data from the SPI bus without sending anything.
    async fn read(&mut self, data: &mut [u8]) -> Result<(), Error> {
        self.async_do_spi_dma_transfer(DmaSlice::from_slice(data), DmaSlice::null())
            .await
    }

    #[inline(always)]
    fn is_done(&self) -> bool {
        self.0.events_end.read().bits() != 0
    }

    #[inline(always)]
    fn reset_done(&mut self) {
        self.0.events_end.write(|w| w);
    }

    // Stop the SPIM
    fn stop() {
        let spim = unsafe { &*T::ptr() };
        spim.tasks_stop.write(|w| unsafe { w.bits(1) });

        while spim.events_stopped.read().bits() == 0 {}

        spim.events_stopped.write(|w| w);
    }
}

impl<T> Drop for Spim<T>
where
    T: Instance,
{
    fn drop(&mut self) {
        Self::stop();
    }
}

mod async_hal_traits {
    use super::*;

    use embedded_hal_async::spi::{SpiBus, SpiBusFlush, SpiBusRead, SpiBusWrite};

    impl<T> SpiBusFlush for Spim<T>
    where
        T: Instance,
    {
        async fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    impl<T> SpiBusRead for Spim<T>
    where
        T: Instance,
    {
        async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            self.read(words).await
        }
    }

    impl<T> SpiBusWrite for Spim<T>
    where
        T: Instance,
    {
        async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            self.write(words).await
        }
    }

    impl<T> SpiBus for Spim<T>
    where
        T: Instance,
    {
        async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
            self.transfer(read, write).await
        }

        async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            self.transfer_in_place(words).await
        }
    }
}

/// GPIO pins for SPIM interface
pub struct Pins {
    /// SPI clock
    pub sck: Pin<Output<PushPull>>,

    /// MOSI Master out, slave in
    /// None if unused
    pub mosi: Option<Pin<Output<PushPull>>>,

    /// MISO Master in, slave out
    /// None if unused
    pub miso: Option<Pin<Input<Floating>>>,
}

#[derive(Debug)]
pub enum Error {
    TxBufferTooLong,
    RxBufferTooLong,
    /// EasyDMA can only read from data memory, read only buffers in flash will fail.
    DMABufferNotInDataMemory,
    Transmit,
    Receive,
}

/// Implemented by all SPIM instances.
pub trait Instance: Deref<Target = spim0::RegisterBlock> + sealed::Sealed {
    fn ptr() -> *const spim0::RegisterBlock;

    fn register_waker(waker: &Waker);

    fn unmask_interrupt();
}

mod sealed {
    pub trait Sealed {}
}

impl sealed::Sealed for SPIM0 {}
impl Instance for SPIM0 {
    fn ptr() -> *const spim0::RegisterBlock {
        SPIM0::ptr()
    }

    fn register_waker(waker: &Waker) {
        SPIM0_WAKER.register(waker);
    }

    fn unmask_interrupt() {
        unsafe {
            cortex_m::peripheral::NVIC::unmask(
                crate::pac::Interrupt::SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0,
            )
        };
    }
}

impl Instance for SPIM1 {
    fn ptr() -> *const spim0::RegisterBlock {
        SPIM1::ptr()
    }

    fn register_waker(waker: &Waker) {
        SPIM1_WAKER.register(waker);
    }

    fn unmask_interrupt() {
        unsafe {
            cortex_m::peripheral::NVIC::unmask(
                crate::pac::Interrupt::SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1,
            )
        };
    }
}
impl sealed::Sealed for SPIM1 {}

impl Instance for SPIM2 {
    fn ptr() -> *const spim0::RegisterBlock {
        SPIM2::ptr()
    }

    fn register_waker(waker: &Waker) {
        SPIM2_WAKER.register(waker);
    }

    fn unmask_interrupt() {
        unsafe { cortex_m::peripheral::NVIC::unmask(crate::pac::Interrupt::SPIM2_SPIS2_SPI2) };
    }
}
impl sealed::Sealed for SPIM2 {}

#[cfg(any(feature = "52833", feature = "52840"))]
mod _spim3 {
    use super::*;
    impl Instance for SPIM3 {
        fn ptr() -> *const spim0::RegisterBlock {
            SPIM3::ptr()
        }
    }
    impl sealed::Sealed for SPIM3 {}
}
