use crate::{
    interrupt_token::InterruptToken, waker_registration::CriticalSectionWakerRegistration,
};
use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};
use nrf52832_hal::{
    gpiote::{EventPolarity, Gpiote, GpioteInputPin},
    pac::GPIOTE,
    prelude::InputPin,
};

#[macro_export]
macro_rules! register_gpiote_interrupt {
    ($token_name:ident) => {
        pub struct $token_name;

        unsafe impl
            async_nrf52832_hal::interrupt_token::InterruptToken<
                async_nrf52832_hal::gpiote::export::GPIOTE,
            > for $token_name
        {
        }

        #[no_mangle]
        #[allow(non_snake_case)]
        unsafe extern "C" fn GPIOTE() {
            async_nrf52832_hal::gpiote::export::on_interrupt_gpiote();
        }
    };
}

// Hidden export only for use by the macro
#[doc(hidden)]
pub mod export {
    pub use nrf52832_hal::pac::GPIOTE;

    /// This happens on interrupt.
    pub fn on_interrupt_gpiote() {
        let gpiote = unsafe { &*GPIOTE::ptr() };

        for ch in 0..8 {
            if gpiote.events_in[ch].read().bits() != 0 {
                gpiote.intenclr.write(|w| unsafe { w.bits(1 << ch) });
                super::WAKER_REGISTRATION[ch].wake();
            }
        }
    }
}

const NW: CriticalSectionWakerRegistration = CriticalSectionWakerRegistration::new();

// Waker registration for each GPIOTE channel.
static WAKER_REGISTRATION: [CriticalSectionWakerRegistration; 8] = [NW; 8];

/// Uninit state for a gpiote channel
pub struct Uninit;

/// Configured state for a gpiote channel with a pin
pub struct Configured<Pin>(Pin);

/// Convert the Gpiote to its channels
pub fn new<IntReg>(
    _gpiote: Gpiote,
    _interrupt_token: impl InterruptToken<GPIOTE>,
) -> (
    Channel<0, Uninit>,
    Channel<1, Uninit>,
    Channel<2, Uninit>,
    Channel<3, Uninit>,
    Channel<4, Uninit>,
    Channel<5, Uninit>,
    Channel<6, Uninit>,
    Channel<7, Uninit>,
) {
    (
        Channel { conf: Uninit },
        Channel { conf: Uninit },
        Channel { conf: Uninit },
        Channel { conf: Uninit },
        Channel { conf: Uninit },
        Channel { conf: Uninit },
        Channel { conf: Uninit },
        Channel { conf: Uninit },
    )
}

/// A Gpiote channel
#[derive(Clone)]
pub struct Channel<const CH: usize, T> {
    conf: T,
}

impl<const CH: usize> Channel<CH, Uninit> {
    /// Configure a channel
    pub fn configure<Pin>(self, input_pin: Pin) -> Channel<CH, Configured<Pin>>
    where
        Pin: GpioteInputPin,
    {
        let gpiote = unsafe { &*GPIOTE::ptr() };

        gpiote.config[CH].write(|w| unsafe { w.psel().bits(input_pin.pin()) });

        // TODO: Is port event needed?
        // gpiote.intenset.write(|w| w.port().set());

        Channel {
            conf: Configured(input_pin),
        }
    }
}

impl<const CH: usize, Pin> Channel<CH, Configured<Pin>>
where
    Pin: GpioteInputPin,
{
    /// Enable interrupts from this channel.
    fn enable_interrupt() {
        // SAFETY: Atomic write.
        unsafe {
            let gpiote = &*GPIOTE::ptr();
            gpiote.intenset.write(|w| w.bits(1 << CH));
        }
    }

    /// Disable interrupts from this channel.
    fn disable_interrupt() {
        // SAFETY: Atomic write.
        unsafe {
            let gpiote = &*GPIOTE::ptr();
            gpiote.intenclr.write(|w| w.bits(1 << CH));
        }
    }

    /// Checks if the event is triggered on this channel.
    fn is_event_triggered() -> bool {
        // SAFETY: Atomic read.
        unsafe {
            let gpiote = &*GPIOTE::ptr();
            gpiote.events_in[CH].read().bits() != 0
        }
    }

    /// Resets the event on this channel.
    fn reset_events() {
        // SAFETY: Atomic write.
        unsafe {
            let gpiote = &*GPIOTE::ptr();
            gpiote.events_in[CH].write(|w| w);
        }
    }

    fn set_trigger(trigger_mode: EventPolarity) {
        let gpiote = unsafe { &*GPIOTE::ptr() };
        gpiote.config[CH].modify(|_, w| match trigger_mode {
            EventPolarity::HiToLo => w.mode().event().polarity().hi_to_lo(),
            EventPolarity::LoToHi => w.mode().event().polarity().lo_to_hi(),
            EventPolarity::None => w.mode().event().polarity().none(),
            EventPolarity::Toggle => w.mode().event().polarity().toggle(),
        });
    }

    /// Main API for waiting on a pin.
    pub async fn wait_for(&mut self, waiting_for: WaitingFor) -> GpioteChannelFuture<CH, Pin> {
        Self::disable_interrupt();
        Self::reset_events();

        GpioteChannelFuture {
            channel: self,
            waiting_for,
        }
    }

    /// Free the channel an pin.
    pub fn free(self) -> (Pin, Channel<CH, Uninit>) {
        Self::reset_events();
        Self::disable_interrupt();

        let gpiote = unsafe { &*GPIOTE::ptr() };
        gpiote.config[CH].write(|w| w);

        (self.conf.0, Channel { conf: Uninit })
    }
}

// A channel's pin can be read like any input pin.
impl<const CH: usize, Pin> InputPin for Channel<CH, Configured<Pin>>
where
    Pin: InputPin,
{
    type Error = <Pin as InputPin>::Error;

    fn is_low(&self) -> Result<bool, Self::Error> {
        self.conf.0.is_low()
    }

    fn is_high(&self) -> Result<bool, Self::Error> {
        self.conf.0.is_high()
    }
}

// Error type for the async traits.
impl<const CH: usize, Pin> embedded_hal::digital::ErrorType for Channel<CH, Configured<Pin>>
where
    Pin: InputPin,
    <Pin as InputPin>::Error: core::fmt::Debug,
{
    type Error = <Pin as InputPin>::Error;
}

impl<const CH: usize, Pin> embedded_hal_async::digital::Wait for Channel<CH, Configured<Pin>>
where
    Pin: GpioteInputPin + InputPin,
    <Pin as InputPin>::Error: core::fmt::Debug,
{
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for(WaitingFor::High).await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for(WaitingFor::Low).await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for(WaitingFor::RisingEdge).await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for(WaitingFor::FallingEdge).await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for(WaitingFor::AnyEdge).await;
        Ok(())
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum WaitingFor {
    High,
    Low,
    RisingEdge,
    FallingEdge,
    AnyEdge,
}

pub struct GpioteChannelFuture<'a, const CH: usize, Pin> {
    channel: &'a mut Channel<CH, Configured<Pin>>,
    waiting_for: WaitingFor,
}

impl<'a, const CH: usize, P> Future for GpioteChannelFuture<'a, CH, P>
where
    P: GpioteInputPin + InputPin,
{
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        match &self.waiting_for {
            WaitingFor::High => {
                // Check fall through
                if matches!(self.channel.conf.0.is_high(), Ok(true)) {
                    Channel::<CH, Configured<P>>::disable_interrupt();
                    return Poll::Ready(());
                }

                Channel::<CH, Configured<P>>::set_trigger(EventPolarity::LoToHi);
            }
            WaitingFor::Low => {
                // Check fall through
                if matches!(self.channel.conf.0.is_low(), Ok(true)) {
                    Channel::<CH, Configured<P>>::disable_interrupt();
                    return Poll::Ready(());
                }

                Channel::<CH, Configured<P>>::set_trigger(EventPolarity::HiToLo);
            }
            WaitingFor::RisingEdge => {
                Channel::<CH, Configured<P>>::set_trigger(EventPolarity::LoToHi);
            }
            WaitingFor::FallingEdge => {
                Channel::<CH, Configured<P>>::set_trigger(EventPolarity::HiToLo);
            }
            WaitingFor::AnyEdge => {
                Channel::<CH, Configured<P>>::set_trigger(EventPolarity::Toggle);
            }
        }

        match &self.waiting_for {
            WaitingFor::RisingEdge | WaitingFor::FallingEdge | WaitingFor::AnyEdge => {
                if Channel::<CH, Configured<P>>::is_event_triggered() {
                    Channel::<CH, Configured<P>>::reset_events();
                    Channel::<CH, Configured<P>>::disable_interrupt();

                    return Poll::Ready(());
                }
            }
            _ => {}
        }

        WAKER_REGISTRATION[CH].register(cx.waker());

        // Enable interrupts after registration
        Channel::<CH, Configured<P>>::enable_interrupt();

        Poll::Pending
    }
}
