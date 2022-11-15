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
        embedded_hal_async::{digital::Wait, spi::SpiBus},
        gpio::{
            self,
            p0::{self, P0_16, P0_17, P0_18, P0_19, P0_20, P0_24, P0_31},
            Floating, Input, OpenDrainConfig, OpenDrainIO, Output, Pin, PullUp, PushPull,
        },
        gpiote,
        pac::SPIM0,
        register_gpiote_interrupt, register_spim0_interrupt,
        spim::{Frequency, Pins, Spim, MODE_0},
    };
    use systick_monotonic::*;

    type ButtonPin = Pin<Input<PullUp>>;
    type LedPin = P0_31<Output<PushPull>>;
    type IrqButton = gpiote::Channel<0, gpiote::Configured<ButtonPin>>;

    type DW1000Clk = P0_16<Output<PushPull>>;
    type DW1000Mosi = P0_20<Output<PushPull>>;
    type DW1000Miso = P0_18<Output<PushPull>>;
    type DW1000Cs = P0_17<Output<PushPull>>;
    type DW1000Irq = P0_19<Input<Floating>>;
    type DW1000Rst = P0_24<Output<OpenDrainIO>>;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<1_000>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: LedPin,
        btn: IrqButton,
        cs: DW1000Cs,
        spi: Spim<SPIM0>,
    }

    register_gpiote_interrupt!(GpioteToken);
    register_spim0_interrupt!(Spim0Token);

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        let _clock = clocks::Clocks::new(cx.device.CLOCK).enable_ext_hfosc();
        let port0 = p0::Parts::new(cx.device.P0);

        let btn = port0.p0_02.into_pullup_input();
        let led = port0.p0_31.into_push_pull_output(gpio::Level::High);

        let (ch0, ..) = gpiote::new(cx.device.GPIOTE, GpioteToken);
        let btn = ch0.configure(btn.degrade());

        let spiclk = port0.p0_16.into_push_pull_output(gpio::Level::Low);
        let spimosi = port0.p0_20.into_push_pull_output(gpio::Level::Low);
        let spimiso = port0.p0_18.into_floating_input();
        let cs = port0.p0_17.into_push_pull_output(gpio::Level::High);
        let _irq = port0.p0_19.into_floating_input();
        let _rst = port0
            .p0_24
            .into_open_drain_input_output(OpenDrainConfig::Standard0Disconnect1, gpio::Level::High);

        let spi = Spim::new(
            cx.device.SPIM0,
            Pins {
                sck: spiclk.degrade(),
                mosi: Some(spimosi.degrade()),
                miso: Some(spimiso.degrade()),
            },
            Frequency::M1,
            MODE_0,
            0,
            Spim0Token,
        );

        let mono = Systick::new(cx.core.SYST, 64_000_000);

        async_task::spawn().ok();

        (
            Shared {},
            Local { led, btn, cs, spi },
            init::Monotonics(mono),
        )
    }

    #[task(local = [led, btn, cs, spi])]
    async fn async_task(cx: async_task::Context) {
        let async_task::LocalResources { led, btn, cs, spi } = cx.local;

        defmt::info!("hello");
        loop {
            btn.wait_for_falling_edge().await.ok();
            defmt::info!("Button pressed");
            led.set_low().ok();

            cs.set_low().ok();

            let mut buf = [0; 5];
            spi.transfer_in_place(&mut buf).await.unwrap();

            cs.set_high().ok();

            defmt::info!("SPI data: {:x}", buf);

            btn.wait_for_rising_edge().await.ok();
            defmt::info!("Button released");
            led.set_high().ok();
        }
    }
}
