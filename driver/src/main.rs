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

pub mod test;

#[rtic::app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [EXTI0, EXTI1, EXTI2]
)]
mod app {

    use core::fmt::Write;

    use super::hal;

    use hal::gpio::Edge;
    use hal::gpio::{NoPin, Pin};
    use hal::i2s::stm32_i2s_v12x::driver::*;
    use hal::i2s::I2s;
    use hal::pac::DWT;
    use hal::pac::{EXTI, SPI2, SPI3};
    use hal::prelude::*;

    use super::test;

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

    // Part of the frame we currently transmit or receive
    #[derive(Copy, Clone)]
    pub enum FrameState {
        LeftMsb,
        LeftLsb,
        RightMsb,
        RightLsb,
    }

    use FrameState::{LeftLsb, LeftMsb, RightLsb, RightMsb};

    impl Default for FrameState {
        fn default() -> Self {
            Self::LeftMsb
        }
    }

    #[derive(Copy, Clone)]
    pub enum I2sCtl {
        Disable,
        Enable,
    }

    #[shared]
    struct Shared {
        i2s2_driver: Option<ReceiveDriver<I2s2, Philips>>,
        i2s3_driver: Option<TransmitDriver<I2s3, Philips>>,
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
        i2s2_data_p: Producer<'static, (u32, (i32, i32)), 8>,
        i2s2_data_c: Consumer<'static, (u32, (i32, i32)), 8>,
        i2s3_data_p: Producer<'static, (i32, i32), 8>,
        i2s3_data_c: Consumer<'static, (i32, i32), 8>,
        i2s2_ctl_p: Producer<'static, I2sCtl, 2>,
        i2s2_ctl_c: Consumer<'static, I2sCtl, 2>,
        i2s3_ctl_p: Producer<'static, I2sCtl, 2>,
        i2s3_ctl_c: Consumer<'static, I2sCtl, 2>,
    }

    #[init(
        local = [
            i2s2_data_q: Queue<(u32, (i32,i32)), 8> = Queue::new(),
            i2s3_data_q: Queue<(i32,i32), 8> = Queue::new(),
            i2s2_ctl_q: Queue<I2sCtl, 2> = Queue::new(),
            i2s3_ctl_q: Queue<I2sCtl, 2> = Queue::new()]
        )]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let i2s2_data_q = cx.local.i2s2_data_q;
        let i2s3_data_q = cx.local.i2s3_data_q;
        let i2s2_ctl_q = cx.local.i2s2_ctl_q;
        let i2s3_ctl_q = cx.local.i2s3_ctl_q;
        let channels = rtt_init! {
            up: {
                0: {
                    size: 2048
                    name: "Logs"
                }
                1: {
                    size: 1024
                    name: "Panics"
                }
            }
        };
        let logs_chan = channels.up.0;
        let panics_chan = channels.up.1;
        set_print_channel(panics_chan);
        let (i2s2_data_p, i2s2_data_c) = i2s2_data_q.split();
        let (i2s3_data_p, i2s3_data_c) = i2s3_data_q.split();
        let (i2s2_ctl_p, i2s2_ctl_c) = i2s2_ctl_q.split();
        let (i2s3_ctl_p, i2s3_ctl_c) = i2s3_ctl_q.split();
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
        let i2s2_driver = None; //Some(ReceiveDriver::Master(i2s2_driver));
        let i2s3_driver = None; //Some(TransmitDriver::Slave(i2s3_driver));

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
                i2s2_data_p,
                i2s2_data_c,
                i2s3_data_p,
                i2s3_data_c,
                i2s2_ctl_p,
                i2s2_ctl_c,
                i2s3_ctl_p,
                i2s3_ctl_c,
            },
            init::Monotonics(),
        )
    }

    #[idle(
        shared = [i2s2_driver, i2s3_driver,exti],
        local = [i2s2, i2s3, i2s2_data_c, i2s3_data_p, i2s2_ctl_p, i2s3_ctl_p]
    )]
    fn idle(cx: idle::Context) -> ! {
        let i2s2 = cx.local.i2s2.take().unwrap();
        let i2s3 = cx.local.i2s3.take().unwrap();
        let i2s2_data_c = cx.local.i2s2_data_c;
        let i2s3_data_p = cx.local.i2s3_data_p;
        //let i2s2_ctl_p = cx.local.i2s2_ctl_p;
        //let i2s3_ctl_p = cx.local.i2s3_ctl_p;
        let mut shared_i2s2_driver = cx.shared.i2s2_driver;
        let mut shared_i2s3_driver = cx.shared.i2s3_driver;
        let mut shared_exti = cx.shared.exti;

        let (i2s2, i2s3) = test::master_receive_slave_transmit_driver_interrupt(
            &mut shared_exti,
            &mut shared_i2s2_driver,
            &mut shared_i2s3_driver,
            i2s2_data_c,
            i2s3_data_p,
            i2s2,
            i2s3,
        );

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
            frame_state: FrameState = LeftMsb,
            frame: (u32, u32) = (0, 0), i2s2_data_p,
            i2s2_ctl_c
        ],
        shared = [i2s2_driver]
    )]
    fn i2s2(cx: i2s2::Context) {
        let frame_state = cx.local.frame_state;
        let frame = cx.local.frame;
        let i2s2_data_p = cx.local.i2s2_data_p;
        let mut i2s2_driver = cx.shared.i2s2_driver;
        i2s2_driver.lock(|i2s2_driver| {
            match i2s2_driver {
                Some(ReceiveDriver::Master(i2s2_driver)) => {
                    let status = i2s2_driver.status();
                    // It's better to read first to avoid triggering ovr flag
                    if status.rxne() {
                        let data = i2s2_driver.read_data_register();
                        match (*frame_state, status.chside()) {
                            (LeftMsb, Channel::Left) => {
                                frame.0 = (data as u32) << 16;
                                *frame_state = LeftLsb;
                            }
                            (LeftLsb, Channel::Left) => {
                                frame.0 |= data as u32;
                                *frame_state = RightMsb;
                            }
                            (RightMsb, Channel::Right) => {
                                frame.1 = (data as u32) << 16;
                                *frame_state = RightLsb;
                            }
                            (RightLsb, Channel::Right) => {
                                frame.1 |= data as u32;
                                // defer sample processing to another task
                                let (l, r) = *frame;
                                i2s2_data_p
                                    .enqueue((DWT::cycle_count(), (l as i32, r as i32)))
                                    .ok();
                                if !i2s2_data_p.ready() {
                                    i2s2_driver.disable();
                                    rprintln!("{} master receive stopped", DWT::cycle_count(),);
                                }
                                *frame_state = LeftMsb;
                            }
                            // in case of ovr this resynchronize at start of new frame
                            _ => {
                                log::spawn(DWT::cycle_count(), "i2s2 Channel Err").ok();
                                *frame_state = LeftMsb;
                            }
                        }
                    }
                    if status.ovr() {
                        log::spawn(DWT::cycle_count(), "i2s2 Overrun").ok();
                        // sequence to delete ovr flag
                        i2s2_driver.read_data_register();
                        i2s2_driver.status();
                    }
                }
                _ => (),
            }
        });
    }

    #[task(
        priority = 4,
        binds = SPI3,
        local = [
            frame_state: FrameState = LeftMsb,
            frame: (u32,u32) = (0,0),i2s3_data_c,
            i2s3_ctl_c
        ],
        shared = [i2s3_driver,exti]
    )]
    fn i2s3(cx: i2s3::Context) {
        let frame_state = cx.local.frame_state;
        let frame = cx.local.frame;
        let i2s3_data_c = cx.local.i2s3_data_c;
        let mut i2s3_driver = cx.shared.i2s3_driver;
        let mut exti = cx.shared.exti;
        i2s3_driver.lock(|i2s3_driver| {
            match i2s3_driver {
                Some(TransmitDriver::Slave(i2s3_driver)) => {
                    let status = i2s3_driver.status();
                    // it's better to write data first to avoid to trigger udr flag
                    if status.txe() {
                        let data;
                        match (*frame_state, status.chside()) {
                            (LeftMsb, Channel::Left) => {
                                let (l, r) = i2s3_data_c.dequeue().unwrap_or_default();
                                *frame = (l as u32, r as u32);
                                data = (frame.0 >> 16) as u16;
                                *frame_state = LeftLsb;
                            }
                            (LeftLsb, Channel::Left) => {
                                data = (frame.0 & 0xFFFF) as u16;
                                *frame_state = RightMsb;
                            }
                            (RightMsb, Channel::Right) => {
                                data = (frame.1 >> 16) as u16;
                                *frame_state = RightLsb;
                            }
                            (RightLsb, Channel::Right) => {
                                data = (frame.1 & 0xFFFF) as u16;
                                *frame_state = LeftMsb;
                            }
                            // in case of udr this resynchronize tracked and actual channel
                            _ => {
                                *frame_state = LeftMsb;
                                data = 0; //garbage data to avoid additional underrrun
                            }
                        }
                        i2s3_driver.write_data_register(data);
                    }
                    if status.fre() {
                        log::spawn(DWT::cycle_count(), "i2s3 Frame error").ok();
                        i2s3_driver.disable();
                        exti.lock(|exti| {
                            i2s3_driver
                                .i2s_peripheral_mut()
                                .ws_pin_mut()
                                .enable_interrupt(exti)
                        })
                    }
                    if status.udr() {
                        log::spawn(DWT::cycle_count(), "i2s3 udr").ok();
                        i2s3_driver.status();
                        i2s3_driver.write_data_register(0);
                    }
                }
                _ => (),
            }
        })
    }

    // Look i2s3 WS line for slave (re) synchronisation
    #[task(priority = 4, binds = EXTI4, shared = [i2s3_driver,exti])]
    fn exti4(cx: exti4::Context) {
        let i2s3_driver = cx.shared.i2s3_driver;
        let exti = cx.shared.exti;
        (exti, i2s3_driver).lock(|exti, i2s3_driver| {
            match i2s3_driver {
                Some(TransmitDriver::Slave(i2s3_driver)) => {
                    let ws_pin = i2s3_driver.i2s_peripheral_mut().ws_pin_mut();
                    ws_pin.clear_interrupt_pending_bit();
                    // yes, in this case we already know that pin is high, but some other exti can be triggered
                    // by several pins
                    if ws_pin.is_high() {
                        ws_pin.disable_interrupt(exti);
                        i2s3_driver.write_data_register(0);
                        i2s3_driver.enable();
                    }
                }
                _ => (),
            }
        });
    }

    // Look i2s2 WS line for slave (re) synchronisation
    #[task(priority = 4, binds = EXTI15_10, shared = [i2s2_driver,exti])]
    fn exti15_10(cx: exti15_10::Context) {
        let i2s2_driver = cx.shared.i2s2_driver;
        let exti = cx.shared.exti;
        (exti, i2s2_driver).lock(|exti, i2s2_driver| {
            match i2s2_driver {
                Some(ReceiveDriver::Slave(i2s2_driver)) => {
                    let ws_pin = i2s2_driver.i2s_peripheral_mut().ws_pin_mut();
                    ws_pin.clear_interrupt_pending_bit();
                    // yes, in this case we already know that pin is high, but some other exti can be triggered
                    // by several pins
                    if ws_pin.is_high() {
                        ws_pin.disable_interrupt(exti);
                        //i2s2_driver.write_data_register(0);
                        i2s2_driver.enable();
                    }
                }
                _ => (),
            }
        });
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
