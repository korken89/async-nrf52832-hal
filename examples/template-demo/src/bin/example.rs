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
    use demo_app::hal::{
        clocks,
        embedded_hal::digital::{InputPin, OutputPin},
        embedded_hal_async::digital::Wait,
        gpio::{
            self,
            p0::{self, P0_02, P0_31},
            Input, Output, Pin, PullUp, PushPull,
        },
        gpiote, register_gpiote_interrupt,
    };
    use systick_monotonic::*;

    type ButtonPin = Pin<Input<PullUp>>;
    type LedPin = P0_31<Output<PushPull>>;
    type IrqButton = gpiote::Channel<0, gpiote::Configured<ButtonPin>>;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<1_000>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: LedPin,
        btn: IrqButton,
    }

    register_gpiote_interrupt!(GpioteToken);

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        let _clock = clocks::Clocks::new(cx.device.CLOCK).enable_ext_hfosc();
        let port0 = p0::Parts::new(cx.device.P0);

        let btn = port0.p0_02.into_pullup_input();
        let led = port0.p0_31.into_push_pull_output(gpio::Level::High);

        let (ch0, ..) = gpiote::new(cx.device.GPIOTE, GpioteToken);
        let btn = ch0.configure(btn.degrade());

        let mono = Systick::new(cx.core.SYST, 64_000_000);

        async_task::spawn().ok();

        (Shared {}, Local { led, btn }, init::Monotonics(mono))
    }

    #[task(local = [led, btn])]
    async fn async_task(cx: async_task::Context) {
        defmt::info!("hello");
        loop {
            cx.local.btn.wait_for_low().await.ok();
            defmt::info!("Button pressed");
            monotonics::delay(50.millis()).await;
        }
    }
}
