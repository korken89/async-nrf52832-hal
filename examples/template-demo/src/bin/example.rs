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
    use demo_app::hal;
    use systick_monotonic::*;
    // use hal::prelude::*;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<1_000>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mono = Systick::new(cx.core.SYST, 64_000_000);
        defmt::info!("init");

        (Shared {}, Local {}, init::Monotonics(mono))
    }

    #[task(local = [])]
    async fn async_task(cx: async_task::Context) {
        // let spi = cx.local.async_spi_handle;
        // let cs = &mut cx.local.dw1000.cs;

        // defmt::info!("delay long time");

        loop {
            monotonics::delay(100.millis()).await;

            // cs.set_low().ok();

            // let mut buf = [0; 5];

            // spi.transfer_in_place(&mut buf).await.ok();

            // defmt::info!("SPI done! Res: {:x}", buf);

            // cs.set_high().ok();
        }

        // defmt::info!("we have just created the future");
        // fut.await;
        // defmt::info!("long delay done");

        // defmt::info!("delay short time");
        // sleep(500.millis()).await;
        // defmt::info!("short delay done");

        // defmt::info!("test timeout");
        // let res = timeout(NeverEndingFuture {}, 1.secs()).await;
        // defmt::info!("timeout done: {:?}", res);

        // defmt::info!("test timeout 2");
        // let res = timeout(Delay::spawn(500.millis()), 1.secs()).await;
        // defmt::info!("timeout done 2: {:?}", res);
    }
}
