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

use stm32_i2s_v12x::I2sPeripheral;
use stm32_i2s_v12x::driver::{DataFormat, *};
use stm32_i2s_v12x::marker;
use stm32_i2s_v12x::transfer::*;
use stm32f4xx_hal::i2s::stm32_i2s_v12x;

use stm32f4xx_hal::gpio::NoPin;
use stm32f4xx_hal::i2s::I2s;
use stm32f4xx_hal::pac::CorePeripherals;
use stm32f4xx_hal::pac::Peripherals;
use stm32f4xx_hal::prelude::*;

macro_rules! test_32bits {
    ($i2s2_transfer:ident, $i2s3_transfer:ident) => {
        let mut count = 0;
        let mut smpls = [(0, 0); 3];
        let frm = [
            (0xDEADBEEF_u32 as _, 0xFEDEDBEE_u32 as _),
            (0xCAFEC0CA_u32 as _, 0xFADEDFEE_u32 as _),
            (0xBADC0C0A_u32 as _, 0xF00DC0DE_u32 as _),
        ];
        $i2s2_transfer.begin();
        $i2s3_transfer.begin();
        while count < 3 {
            $i2s2_transfer.write(frm[count]).ok();

            if let Ok(s) = $i2s3_transfer.read() {
                smpls[count] = s;
                count += 1;
            }
        }

        for (i,(e, s)) in frm.iter().zip(smpls.iter()).enumerate() {
            let msg = if i == 0 {
                "Ignored"
            } else if e == s {
                "Ok"
            } else {
                "Fail"
            };
            rprintln!(
                "transmitted: {:08x} {:08x}, received: {:08x} {:08x}, {}",
                e.0 as u32,
                e.1 as u32,
                s.0 as u32,
                s.1 as u32,
                msg
            );
        }
    }
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
    let mut i2s2 = Some(I2s::new(dp.SPI2, i2s2_pins, &clocks));

    let i2s3_pins = (gpioa.pa4, gpioc.pc10, NoPin, gpioc.pc12);
    let mut i2s3 = Some(I2s::new(dp.SPI3, i2s3_pins, &clocks));

    let i2s5_pins = (gpiob.pb1, gpiob.pb0, NoPin, gpiob.pb8);
    let mut i2s5 = Some(I2s::new(dp.SPI5, i2s5_pins, &clocks));

    let i2s2_transfer_conf = I2sTransferConfig::new_master()
        .transmit()
        .standard(marker::Philips)
        .data_format(marker::Data32Channel32)
        .master_clock(true)
        .request_frequency(1); //try to get the lowest possible sample rate
    let i2s3_transfer_conf = i2s2_transfer_conf.to_slave().receive();
    let i2s5_driver_conf = I2sDriverConfig::new_master()
        .receive()
        .standard(Philips)
        .data_format(DataFormat::Data32Channel32)
        .master_clock(true)
        .request_frequency(1);
        //.i2s_driver(i2s5.take().unwrap());

    let mut i2s2_transfer = i2s2_transfer_conf.i2s_transfer(i2s2.take().unwrap());
    let mut i2s3_transfer = i2s3_transfer_conf.i2s_transfer(i2s3.take().unwrap());

    rprintln!("master transmit + slave receive 32bits");
    test_32bits!(i2s2_transfer, i2s3_transfer);
    loop{}
}

use core::panic::PanicInfo;
#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
