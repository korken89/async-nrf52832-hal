#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]

use demo_app as _; // global logger + panicking-behavior + memory layout

defmt::timestamp!("{=u64:us}", {
    let time_us: systick_monotonic::fugit::MicrosDurationU64 =
        app::monotonics::now().duration_since_epoch().convert();

    time_us.ticks()
});

#[rtic::app(device = demo_app::hal::pac, dispatchers = [SWI0_EGU0])]
mod app {
    use async_nrf52832_hal::gpio::Floating;
    use demo_app::hal::{
        clocks,
        gpio::{
            p0::{self, P0_04},
            Input,
        },
        pac, register_saadc_interrupt,
        saadc::*,
    };
    use systick_monotonic::*;

    type AdcPin = P0_04<Input<Floating>>;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<1_000>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        adc: Saadc,
        pin: AdcPin,
    }

    register_saadc_interrupt!(SaadcToken);

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        let _clock = clocks::Clocks::new(cx.device.CLOCK).enable_ext_hfosc();
        let port0 = p0::Parts::new(cx.device.P0);

        port0
            .p0_05
            .into_push_pull_output(async_nrf52832_hal::gpio::Level::High);

        let pin = port0.p0_04.into_floating_input();
        let adc = Saadc::new(
            cx.device.SAADC,
            SaadcConfig {
                time: Time::_40US,
                reference: Reference::INTERNAL,
                ..Default::default()
            },
            SaadcToken,
        );

        let mono = Systick::new(cx.core.SYST, 64_000_000);

        async_task::spawn().ok();

        (Shared {}, Local { adc, pin }, init::Monotonics(mono))
    }

    #[task(local = [adc, pin])]
    async fn async_task(cx: async_task::Context) {
        let adc = cx.local.adc;
        let pin = cx.local.pin;

        defmt::info!("hello");

        // cortex_m::peripheral::NVIC::pend(pac::interrupt::SAADC);

        loop {
            adc.calibrate().await;

            let val = adc.read(pin).await;
            defmt::info!("Adc value: {}", val);
            monotonics::delay(100.millis()).await;
        }
    }
}
