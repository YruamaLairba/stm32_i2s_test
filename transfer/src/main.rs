//! # I2S test transfer
//!
//! Testing transfer module by sending and receive data under different configuration.
//!
//! # Hardware Wiring
//!
//! This use several SPI/I2S peripheral of the chip connected together.

#![no_std]
#![no_main]
use cortex_m_rt::entry;

use rtt_target::{rprint, rprintln, rtt_init_print};

use stm32_i2s_v12x::driver::{DataFormat, *};
use stm32_i2s_v12x::marker;
use stm32_i2s_v12x::transfer::*;
use stm32_i2s_v12x::I2sPeripheral;
use stm32f4xx_hal::i2s::stm32_i2s_v12x;

use stm32f4xx_hal::gpio::NoPin;
use stm32f4xx_hal::i2s::I2s;
use stm32f4xx_hal::pac::CorePeripherals;
use stm32f4xx_hal::pac::Peripherals;
use stm32f4xx_hal::pac::{RCC, SPI2, SPI3, SPI5};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::Reset;

macro_rules! test_32bits {
    ($i2s2_transfer:ident, $i2s3_transfer:ident) => {
        let mut rcount = 0;
        let mut wcount = 0;
        let mut smpls = [(0x88888888u32 as _ , 0x88888888u32 as _); 4];
        let frm = [
            (0x55555555_u32 as _, 0xFFFFFFFF_u32 as _),
            (0x22222222_u32 as _, 0x33333333_u32 as _),
            (0xAAAABBBB_u32 as _, 0xCCCCDDDD_u32 as _),
            (0x44446666_u32 as _, 0x77779999_u32 as _),
        ];
        loop {
            if wcount <= 3 {
                if $i2s2_transfer.write(frm[wcount]).is_ok() {
                    wcount += 1;
                }
            } else {
                $i2s2_transfer.write((0xC3C3C3C3u32 as _,0xC3C3C3C3u32 as _)).ok();
            }

            if let Ok(s) = $i2s3_transfer.read() {
                if rcount <= 3 {
                    smpls[rcount] = s;
                    rcount += 1;
                } else {
                    break
                }
            }
        }

        for (i, (e, s)) in frm.iter().zip(smpls.iter()).enumerate() {
            let msg = if i == 0 {
                "Ignored"
            } else if *e == *s {
                "Ok"
            } else {
                "Fail"
            };
            //rprintln!("{:?} {:x} {:x}, {:?} {:x} {:x}, {}", *e,e.0,e.1,*s,s.0,s.1,*e==*s);
            rprintln!(
                "transmitted: {:08x} {:08x}, received: {:08x} {:08x}, {}",
                e.0 as u32,
                e.1 as u32,
                s.0 as u32,
                s.1 as u32,
                msg
            );
        }
    };
}

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
        .sysclk(77.MHz())
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

    let transfer_base_conf = I2sTransferConfig::new_master()
        .receive()
        .standard(marker::Philips)
        .data_format(marker::Data32Channel32)
        .master_clock(true)
        .request_frequency(1); //try to get the lowest possible sample rate
    let driver_base_conf = I2sDriverConfig::new_master()
        .receive()
        .standard(Philips)
        .data_format(DataFormat::Data32Channel32)
        .master_clock(true)
        .request_frequency(1);
    //.i2s_driver(i2s5.take().unwrap());

    rprintln!("master transmit + slave receive 32bits");
    let mut i2s2_transfer = transfer_base_conf.transmit().i2s_transfer(i2s2);
    let mut i2s3_transfer = transfer_base_conf.to_slave().receive().i2s_transfer(i2s3);
    test_32bits!(i2s2_transfer, i2s3_transfer);
    let i2s2 = i2s2_transfer.release();
    let i2s3 = i2s3_transfer.release();
    unsafe {
        let rcc = &(*RCC::ptr());
        SPI2::reset(rcc);
        SPI3::reset(rcc);
    }

    rprintln!("slave transmit + master receive 32bits");
    let mut i2s2_transfer = transfer_base_conf.to_slave().transmit().i2s_transfer(i2s2);
    let mut i2s3_transfer = transfer_base_conf.receive().i2s_transfer(i2s3);
    i2s2_transfer.begin();
    test_32bits!(i2s2_transfer, i2s3_transfer);
    let i2s2 = i2s2_transfer.release();
    let i2s3 = i2s3_transfer.release();
    unsafe {
        let rcc = &(*RCC::ptr());
        SPI2::reset(rcc);
        SPI3::reset(rcc);
    }

    rprintln!("slave transmit + slave receive 32bits (external master)");
    let mut i2s5_driver = driver_base_conf.i2s_driver(i2s5);
    let mut i2s2_transfer = transfer_base_conf.to_slave().transmit().i2s_transfer(i2s2);
    let mut i2s3_transfer = transfer_base_conf.to_slave().receive().i2s_transfer(i2s3);
    i2s2_transfer.begin();
    i2s3_transfer.begin();
    i2s5_driver.enable();
    test_32bits!(i2s2_transfer, i2s3_transfer);
    let i2s2 = i2s2_transfer.release();
    let i2s3 = i2s3_transfer.release();
    unsafe {
        let rcc = &(*RCC::ptr());
        SPI2::reset(rcc);
        SPI3::reset(rcc);
    }

    loop {}
}

use core::panic::PanicInfo;
#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
