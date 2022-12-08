#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]

use demo_app as _; // global logger + panicking-behavior + memory layout

defmt::timestamp!("{=u64:us}", {
    let time_us: demo_app::hal::monotonic::fugit::MicrosDurationU32 =
        app::monotonics::now().duration_since_epoch().convert();

    time_us.ticks().into()
});

#[rtic::app(device = demo_app::hal::pac, dispatchers = [SWI0_EGU0])]
mod app {
    use async_nrf52832_hal::{gpio::Floating, pac::TIMER0};
    use demo_app::hal::{
        clocks,
        gpio::{
            p0::{self, P0_04},
            Input,
        },
        monotonic::*,
        register_saadc_interrupt,
        saadc::*,
    };

    type AdcPin = P0_04<Input<Floating>>;

    #[monotonic(binds = TIMER0, default = true)]
    type Mono = MonoTimer<TIMER0>;

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

        cx.device.POWER.tasks_lowpwr.write(|w| unsafe { w.bits(1) });

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

        let mono = MonoTimer::new(cx.device.TIMER0);

        async_task::spawn().ok();

        (Shared {}, Local { pin, adc }, init::Monotonics(mono))
    }

    #[task(local = [adc, pin])]
    async fn async_task(cx: async_task::Context) {
        let adc = cx.local.adc;
        let pin = cx.local.pin;

        defmt::info!("hello");

        loop {
            adc.calibrate().await;

            let val = adc.read(pin).await;
            defmt::info!("Adc value: {}", val);
            monotonics::delay(100.millis()).await;
        }
    }
}
