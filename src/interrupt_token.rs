// /// This trait is implemented by the HAL.
// pub trait InterruptRegistration<Vector> {
//     const VECTOR: Vector; // Holds vector name for compile-time errors

//     fn on_interrupt();
// }

pub unsafe trait InterruptToken<Periperhal> {}
