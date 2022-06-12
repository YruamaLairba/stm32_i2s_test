//! # I2S test transfer
//!
//! Testing transfer module by sending and receive data under different configuration.
//!
//! # Hardware Wiring
//!
//! This use several SPI/I2S peripheral of the chip connected together.

#![no_std]
#![no_main]
use core::cell::Cell;
use cortex_m::interrupt::{free as interrupt_free, Mutex};
use cortex_m_rt::entry;

use rtt_target::{rprint, rprintln, rtt_init_print};

use stm32_i2s_v12x::driver::{DataFormat, *};
use stm32_i2s_v12x::marker;
use stm32_i2s_v12x::transfer::*;
use stm32_i2s_v12x::I2sPeripheral;
use stm32f4xx_hal::i2s::stm32_i2s_v12x;

use stm32f4xx_hal::gpio::{NoPin, Pin};
use stm32f4xx_hal::i2s::I2s;
use stm32f4xx_hal::pac::CorePeripherals;
use stm32f4xx_hal::pac::Peripherals;
use stm32f4xx_hal::pac::{RCC, SPI2, SPI3, SPI5};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::Reset;

use stm32f4xx_hal::interrupt;

pub enum TransmitDriver<I, STD> {
    Master(I2sDriver<I, Master, Transmit, STD>),
    Slave(I2sDriver<I, Slave, Transmit, STD>),
}

pub enum ReceiveDriver<I, STD> {
    Master(I2sDriver<I, Master, Receive, STD>),
    Slave(I2sDriver<I, Slave, Receive, STD>),
}

type I2s2 = I2s<
    SPI2,
    (
        Pin<'B', 12_u8>,
        Pin<'B', 13_u8>,
        Pin<'C', 6_u8>,
        Pin<'B', 15_u8>,
    ),
>;
type I2s3 = I2s<SPI3, (Pin<'A', 4_u8>, Pin<'C', 10_u8>, NoPin, Pin<'C', 12_u8>)>;

static T_DRV: Mutex<Cell<Option<TransmitDriver<I2s2, Philips>>>> = Mutex::new(Cell::new(None));
static R_DRV: Mutex<Cell<Option<ReceiveDriver<I2s3, Philips>>>> = Mutex::new(Cell::new(None));

const DRV_FRM: &[u16] = &[
    0x1111, 0xEEEE, 0x2222, 0xDDDD, 0x4444, 0xBBBB, 0x8888, 0x7777,
];
#[entry]
fn main() -> ! {
    rtt_init_print!();
    let dp = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8u32.MHz())
        .sysclk(96.MHz())
        .pclk2(10.MHz())
        .i2s_clk(15.MHz()) // lowest possible speed
        .freeze();
    let mut delay = core.SYST.delay(&clocks);

    // I2S pins: (WS, CK, MCLK, SD)
    let i2s2_pins = (gpiob.pb12, gpiob.pb13, gpioc.pc6, gpiob.pb15);
    let mut i2s2 = I2s::new(dp.SPI2, i2s2_pins, &clocks);

    let i2s3_pins = (gpioa.pa4, gpioc.pc10, NoPin, gpioc.pc12);
    let mut i2s3 = I2s::new(dp.SPI3, i2s3_pins, &clocks);

    let i2s5_pins = (gpiob.pb1, gpiob.pb0, NoPin, gpiob.pb8);
    let mut i2s5 = I2s::new(dp.SPI5, i2s5_pins, &clocks);

    let driver_base_conf = I2sDriverConfig::new_master()
        .receive()
        .standard(Philips)
        .data_format(DataFormat::Data32Channel32)
        .master_clock(true)
        .request_frequency(1);
    //.i2s_driver(i2s5.take().unwrap());

    rprintln!("master transmit + slave receive 32bits");
    let mut i2s2_driver = driver_base_conf.transmit().i2s_driver(i2s2);
    let mut i2s3_driver = driver_base_conf.to_slave().receive().i2s_driver(i2s3);
    let buf = [0u16, 8];
    i2s3_driver.enable();
    i2s2_driver.enable();
    interrupt_free(|cs| {
        i2s2_driver.set_tx_interrupt(true);
        i2s3_driver.set_rx_interrupt(true);
        i2s3_driver.set_error_interrupt(true);
        T_DRV
            .borrow(cs)
            .set(Some(TransmitDriver::Master(i2s2_driver)));
        R_DRV
            .borrow(cs)
            .set(Some(ReceiveDriver::Slave(i2s3_driver)));
    });

    let (w, r) = (0, 0);

    //let i2s2 = i2s2_driver.release();
    //let i2s3 = i2s3_driver.release();
    unsafe {
        let rcc = &(*RCC::ptr());
        SPI2::reset(rcc);
        SPI3::reset(rcc);
    }

    loop {}
}

#[interrupt]
fn SPI2(){
    static mut I2S2 : Option<TransmitDriver<I2s2,Philips>> = None;
    if I2S2.is_none() {
        interrupt_free(|cs| *I2S2 = T_DRV .borrow(cs).take());
    }
}


use core::panic::PanicInfo;
#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
