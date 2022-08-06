//! # I2S test
//!
//! Testing by sending and receive data under different configuration.
//!
//! # Hardware Wiring
//!
//! This use several SPI/I2S peripheral of the chip connected together.

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use rtt_target::rprintln;

use stm32f4xx_hal as hal;

pub mod driver_wrap;
pub mod test;
pub mod tests_16bits;

#[rtic::app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [EXTI0, EXTI1, EXTI2]
)]
mod app {
    use super::*;

    use core::fmt::Write;

    use hal::gpio::Edge;
    use hal::gpio::{NoPin, Pin};
    use hal::i2s::stm32_i2s_v12x::driver::*;
    use hal::i2s::I2s;
    #[allow(unused)]
    use hal::pac::DWT;
    use hal::pac::{EXTI, SPI2, SPI3};
    use hal::prelude::*;

    use driver_wrap::*;

    use heapless::spsc::*;

    use rtt_target::{rprintln, rtt_init, set_print_channel};

    pub const FRM_32: &[(i32, i32)] = &[
        (0x11113333u32 as _, 0x7777EEEEu32 as _),
        (0x22224444u32 as _, 0x55556666u32 as _),
        (0x88889999u32 as _, 0xAAAABBBBu32 as _),
        (0xCCCCDDDDu32 as _, 0x10002000u32 as _),
        (0x30004000u32 as _, 0x50006000u32 as _),
        (0x70008000u32 as _, 0x9000A000u32 as _),
        (0xB000C000u32 as _, 0xD000E000u32 as _),
        (0x01234567u32 as _, 0x89ABCDEFu32 as _),
    ];

    pub enum TransmitDriver<I, STD> {
        Master(I2sDriver<I, Master, Transmit, STD>),
        Slave(I2sDriver<I, Slave, Transmit, STD>),
    }

    pub enum ReceiveDriver<I, STD> {
        Master(I2sDriver<I, Master, Receive, STD>),
        Slave(I2sDriver<I, Slave, Receive, STD>),
    }

    pub type I2s2 = I2s<
        SPI2,
        (
            Pin<'B', 12_u8>,
            Pin<'B', 13_u8>,
            Pin<'C', 6_u8>,
            Pin<'B', 15_u8>,
        ),
    >;
    pub type I2s3 = I2s<SPI3, (Pin<'A', 4_u8>, Pin<'C', 10_u8>, NoPin, Pin<'C', 12_u8>)>;

    #[derive(Copy, Clone)]
    pub enum I2sCtl {
        Disable,
        Enable,
    }

    #[shared]
    struct Shared {
        i2s2_driver: DriverWrap<I2s2>,
        i2s3_driver: DriverWrap<I2s3>,
        exti: EXTI,
    }
    pub use crate::app::shared_resources::exti_that_needs_to_be_locked;
    pub use crate::app::shared_resources::i2s2_driver_that_needs_to_be_locked;
    pub use crate::app::shared_resources::i2s3_driver_that_needs_to_be_locked;

    #[local]
    struct Local {
        logs_chan: rtt_target::UpChannel,
        i2s2: Option<I2s2>,
        i2s3: Option<I2s3>,
        i2s2_data_16_p: Producer<'static, (u32, (i16, i16)), 8>,
        i2s2_data_16_c: Consumer<'static, (u32, (i16, i16)), 8>,
        i2s3_data_16_p: Producer<'static, (i16, i16), 8>,
        i2s3_data_16_c: Consumer<'static, (i16, i16), 8>,
        i2s2_data_32_p: Producer<'static, (u32, (i32, i32)), 8>,
        i2s2_data_32_c: Consumer<'static, (u32, (i32, i32)), 8>,
        i2s3_data_32_p: Producer<'static, (i32, i32), 8>,
        i2s3_data_32_c: Consumer<'static, (i32, i32), 8>,
    }

    #[init(
        local = [
            i2s2_data_16_q: Queue<(u32, (i16,i16)), 8> = Queue::new(),
            i2s3_data_16_q: Queue<(i16,i16), 8> = Queue::new(),
            i2s2_data_32_q: Queue<(u32, (i32,i32)), 8> = Queue::new(),
            i2s3_data_32_q: Queue<(i32,i32), 8> = Queue::new(),
            i2s2_ctl_q: Queue<I2sCtl, 2> = Queue::new(),
            i2s3_ctl_q: Queue<I2sCtl, 2> = Queue::new()]
        )]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let i2s2_data_16_q = cx.local.i2s2_data_16_q;
        let i2s3_data_16_q = cx.local.i2s3_data_16_q;
        let i2s2_data_32_q = cx.local.i2s2_data_32_q;
        let i2s3_data_32_q = cx.local.i2s3_data_32_q;
        let channels = rtt_init! {
            up: {
                0: {
                    size: 128
                    name: "Logs"
                }
                1: {
                    size: 1024
                    mode: BlockIfFull
                    name: "Panics"
                }
            }
        };
        let logs_chan = channels.up.0;
        let panics_chan = channels.up.1;
        set_print_channel(panics_chan);
        let (i2s2_data_16_p, i2s2_data_16_c) = i2s2_data_16_q.split();
        let (i2s3_data_16_p, i2s3_data_16_c) = i2s3_data_16_q.split();
        let (i2s2_data_32_p, i2s2_data_32_c) = i2s2_data_32_q.split();
        let (i2s3_data_32_p, i2s3_data_32_c) = i2s3_data_32_q.split();
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.set_cycle_count(0);
        core.DWT.enable_cycle_counter();
        let device = cx.device;
        let mut syscfg = device.SYSCFG.constrain();
        let mut exti = device.EXTI;

        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
        let gpioc = device.GPIOC.split();
        let rcc = device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8u32.MHz())
            .sysclk(96.MHz())
            .hclk(96.MHz())
            .pclk1(50.MHz())
            .pclk2(100.MHz())
            .i2s_clk(61440.kHz())
            .freeze();

        // I2S pins: (WS, CK, MCLK, SD) for I2S2
        let mut i2s2_pins = (
            gpiob.pb12, //WS
            gpiob.pb13, //CK
            gpioc.pc6,  //MCK
            gpiob.pb15, //SD
        );
        // set up an interrupt on WS pin
        i2s2_pins.0.make_interrupt_source(&mut syscfg);
        i2s2_pins.0.trigger_on_edge(&mut exti, Edge::Rising);
        let i2s2 = Some(I2s::new(device.SPI2, i2s2_pins, &clocks));

        // I2S3 pins: (WS, CK, NoPin, SD) for I2S3
        let mut i2s3_pins = (gpioa.pa4, gpioc.pc10, NoPin, gpioc.pc12);
        // set up an interrupt on WS pin
        i2s3_pins.0.make_interrupt_source(&mut syscfg);
        i2s3_pins.0.trigger_on_edge(&mut exti, Edge::Rising);
        let i2s3 = Some(I2s::new(device.SPI3, i2s3_pins, &clocks));

        //i2s2_driver.enable();
        let i2s2_driver = DriverWrap::new(None); //Some(ReceiveDriver::Master(i2s2_driver));
        let i2s3_driver = DriverWrap::new(None); //Some(TransmitDriver::Slave(i2s3_driver));

        (
            Shared {
                i2s2_driver,
                i2s3_driver,
                exti,
            },
            Local {
                logs_chan,
                i2s2,
                i2s3,
                i2s2_data_16_p,
                i2s2_data_16_c,
                i2s3_data_16_p,
                i2s3_data_16_c,
                i2s2_data_32_p,
                i2s2_data_32_c,
                i2s3_data_32_p,
                i2s3_data_32_c,
            },
            init::Monotonics(),
        )
    }

    #[idle(
        shared = [i2s2_driver, i2s3_driver,exti],
        local = [i2s2, i2s3, i2s2_data_16_c, i2s3_data_16_p, i2s2_data_32_c, i2s3_data_32_p]
    )]
    fn idle(cx: idle::Context) -> ! {
        let i2s2 = cx.local.i2s2.take().unwrap();
        let i2s3 = cx.local.i2s3.take().unwrap();
        let i2s2_data_16_c = cx.local.i2s2_data_16_c;
        let i2s3_data_16_p = cx.local.i2s3_data_16_p;
        let i2s2_data_32_c = cx.local.i2s2_data_32_c;
        let i2s3_data_32_p = cx.local.i2s3_data_32_p;
        //let i2s2_ctl_p = cx.local.i2s2_ctl_p;
        //let i2s3_ctl_p = cx.local.i2s3_ctl_p;
        let mut shared_i2s2_driver = cx.shared.i2s2_driver;
        let mut shared_i2s3_driver = cx.shared.i2s3_driver;
        let mut shared_exti = cx.shared.exti;

        //#[cfg(FALSE)]
        let (i2s2, i2s3) = test::master_receive_slave_transmit_driver_interrupt(
            &mut shared_exti,
            &mut shared_i2s2_driver,
            &mut shared_i2s3_driver,
            i2s2_data_32_c,
            i2s3_data_32_p,
            i2s2,
            i2s3,
        );

        //#[cfg(FALSE)]
        let (i2s2, i2s3) = test::slave_receive_master_transmit_driver_interrupt(
            &mut shared_exti,
            &mut shared_i2s2_driver,
            &mut shared_i2s3_driver,
            i2s2_data_32_c,
            i2s3_data_32_p,
            i2s2,
            i2s3,
        );

        //#[cfg(FALSE)]
        let (i2s2, i2s3) = test::master_transmit_transfer_block(
            &mut shared_exti,
            &mut shared_i2s2_driver,
            i2s2_data_32_c,
            i2s2,
            i2s3,
        );

        //#[cfg(FALSE)]
        let (i2s2, i2s3) = test::master_transmit_transfer_nb(
            &mut shared_exti,
            &mut shared_i2s2_driver,
            i2s2_data_32_c,
            i2s2,
            i2s3,
        );

        #[cfg(FALSE)]
        let (i2s2, i2s3) = test::slave_transmit_transfer_block(
            &mut shared_i2s2_driver,
            i2s2_data_32_c,
            i2s2,
            i2s3,
        );

        #[cfg(FALSE)]
        let (i2s2, i2s3) =
            test::slave_transmit_transfer_nb(&mut shared_i2s2_driver, i2s2_data_32_c, i2s2, i2s3);

        #[cfg(FALSE)]
        let (i2s2, i2s3) = test::master_receive_transfer_block(
            &mut shared_exti,
            &mut shared_i2s3_driver,
            i2s3_data_32_p,
            i2s2,
            i2s3,
        );

        #[cfg(FALSE)]
        let (i2s2, i2s3) = test::master_receive_transfer_nb(
            &mut shared_exti,
            &mut shared_i2s3_driver,
            i2s3_data_32_p,
            i2s2,
            i2s3,
        );

        #[cfg(FALSE)]
        let (i2s2, i2s3) =
            test::slave_receive_transfer_block(&mut shared_i2s3_driver, i2s3_data_32_p, i2s2, i2s3);

        #[cfg(FALSE)]
        let (i2s2, i2s3) =
            test::slave_receive_transfer_nb(&mut shared_i2s3_driver, i2s3_data_32_p, i2s2, i2s3);

        //#[cfg(FALSE)]
        let (i2s2, i2s3) = tests_16bits::master_receive_slave_transmit_driver_interrupt(
            &mut shared_exti,
            &mut shared_i2s2_driver,
            &mut shared_i2s3_driver,
            i2s2_data_16_c,
            i2s3_data_16_p,
            i2s2,
            i2s3,
        );

        //#[cfg(FALSE)]
        let (i2s2, i2s3) = tests_16bits::slave_receive_master_transmit_driver_interrupt(
            &mut shared_exti,
            &mut shared_i2s2_driver,
            &mut shared_i2s3_driver,
            i2s2_data_16_c,
            i2s3_data_16_p,
            i2s2,
            i2s3,
        );

        //#[cfg(FALSE)]
        let (i2s2, i2s3) = tests_16bits::master_transmit_transfer_block(
            &mut shared_exti,
            &mut shared_i2s2_driver,
            i2s2_data_16_c,
            i2s2,
            i2s3,
        );

        //#[cfg(FALSE)]
        let (i2s2, i2s3) = tests_16bits::master_transmit_transfer_nb(
            &mut shared_exti,
            &mut shared_i2s2_driver,
            i2s2_data_16_c,
            i2s2,
            i2s3,
        );

        #[cfg(FALSE)]
        let (i2s2, i2s3) = tests_16bits::slave_transmit_transfer_block(
            &mut shared_i2s2_driver,
            i2s2_data_16_c,
            i2s2,
            i2s3,
        );

        #[cfg(FALSE)]
        let (i2s2, i2s3) = tests_16bits::slave_transmit_transfer_nb(
            &mut shared_i2s2_driver,
            i2s2_data_16_c,
            i2s2,
            i2s3,
        );

        #[cfg(FALSE)]
        let (i2s2, i2s3) = tests_16bits::master_receive_transfer_block(
            &mut shared_exti,
            &mut shared_i2s3_driver,
            i2s3_data_16_p,
            i2s2,
            i2s3,
        );

        #[cfg(FALSE)]
        let (i2s2, i2s3) = tests_16bits::master_receive_transfer_nb(
            &mut shared_exti,
            &mut shared_i2s3_driver,
            i2s3_data_16_p,
            i2s2,
            i2s3,
        );

        #[cfg(FALSE)]
        let (i2s2, i2s3) = tests_16bits::slave_receive_transfer_block(
            &mut shared_i2s3_driver,
            i2s3_data_16_p,
            i2s2,
            i2s3,
        );

        #[cfg(FALSE)]
        let (i2s2, i2s3) = tests_16bits::slave_receive_transfer_nb(
            &mut shared_i2s3_driver,
            i2s3_data_16_p,
            i2s2,
            i2s3,
        );

        let _ = (i2s2, i2s3);
        rprintln!("--- End of Tests");
        #[allow(clippy::empty_loop)]
        loop {}
    }

    // Printing message directly in a i2s interrupt can cause timing issues.
    #[task(capacity = 10, local = [logs_chan])]
    fn log(cx: log::Context, time: u32, msg: &'static str) {
        writeln!(cx.local.logs_chan, "{} {}", time, msg).unwrap();
    }

    #[task(
        priority = 4,
        binds = SPI2,
        local = [
            i2s2_data_16_p,
            i2s2_data_32_p,
        ],
        shared = [i2s2_driver,exti]
    )]
    fn i2s2(cx: i2s2::Context) {
        let i2s2_data_16_p = cx.local.i2s2_data_16_p;
        let i2s2_data_32_p = cx.local.i2s2_data_32_p;
        let mut i2s2_driver = cx.shared.i2s2_driver;
        let mut exti = cx.shared.exti;
        i2s2_driver.lock(|i2s2_driver| {
            i2s2_driver.receive_interrupt_handler(&mut exti, i2s2_data_16_p, i2s2_data_32_p);
        });
    }

    #[task(
        priority = 4,
        binds = SPI3,
        local = [
            i2s3_data_16_c,
            i2s3_data_32_c,
        ],
        shared = [i2s3_driver,exti]
    )]
    fn i2s3(cx: i2s3::Context) {
        let i2s3_data_16_c = cx.local.i2s3_data_16_c;
        let i2s3_data_32_c = cx.local.i2s3_data_32_c;
        let mut i2s3_driver = cx.shared.i2s3_driver;
        let mut exti = cx.shared.exti;
        i2s3_driver.lock(|i2s3_driver| {
            i2s3_driver.transmit_interrupt_handler(&mut exti, i2s3_data_16_c, i2s3_data_32_c);
        })
    }

    // Look i2s3 WS line for slave (re) synchronisation
    #[task(priority = 4, binds = EXTI4, shared = [i2s3_driver,exti])]
    fn exti4(cx: exti4::Context) {
        let i2s3_driver = cx.shared.i2s3_driver;
        let exti = cx.shared.exti;
        (exti, i2s3_driver).lock(|exti, i2s3_driver| {
            i2s3_driver.transmit_exti_handler(exti);
        });
    }

    // Look i2s2 WS line for slave (re) synchronisation
    #[task(priority = 4, binds = EXTI15_10, shared = [i2s2_driver,exti])]
    fn exti15_10(cx: exti15_10::Context) {
        let i2s2_driver = cx.shared.i2s2_driver;
        let exti = cx.shared.exti;
        (exti, i2s2_driver).lock(|exti, i2s2_driver| {
            i2s2_driver.receive_exti_handler(exti);
        });
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
