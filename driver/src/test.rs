//! Contains test to be done

use crate::app::exti_that_needs_to_be_locked;
use crate::app::i2s2_driver_that_needs_to_be_locked;
use crate::app::i2s3_driver_that_needs_to_be_locked;
use crate::app::{I2s2, I2s3};
use crate::app::{ReceiveDriver, TransmitDriver};
use heapless::spsc::*;
use rtt_target::rprintln;

use crate::hal;

use hal::gpio::ExtiPin;
use hal::i2s::stm32_i2s_v12x::driver::*;
use hal::pac::DWT;

use rtic::mutex::prelude::*;

const FRM_32: &[(i32, i32)] = &[
    (0x11113333u32 as _, 0x7777EEEEu32 as _),
    (0x22224444u32 as _, 0x55556666u32 as _),
    (0x88889999u32 as _, 0xAAAABBBBu32 as _),
    (0xCCCCDDDDu32 as _, 0x10002000u32 as _),
    (0x30004000u32 as _, 0x50006000u32 as _),
    (0x70008000u32 as _, 0x9000A000u32 as _),
    (0xB000C000u32 as _, 0xD000E000u32 as _),
    (0x01234567u32 as _, 0x89ABCDEFu32 as _),
];

pub fn master_receive_slave_transmit_driver_interrupt(
    mut shared_exti: &mut exti_that_needs_to_be_locked,
    mut shared_i2s2_driver: &mut i2s2_driver_that_needs_to_be_locked,
    mut shared_i2s3_driver: &mut i2s3_driver_that_needs_to_be_locked,
    i2s2_data_c: &mut Consumer<'static, (u32, (i32, i32)), 8_usize>,
    i2s3_data_p: &mut Producer<'static, (i32, i32), 8_usize>,
    i2s2: I2s2,
    i2s3: I2s3,
) -> (I2s2, I2s3) {
    let mut res_32 = [(0, (0, 0)); 7];

    rprintln!(
        "--- {} Master Receive + Slave transmit driver (interrupt)",
        DWT::cycle_count()
    );

    // Set up drivers
    let mut i2s2_driver = I2sDriverConfig::new_master()
        .receive()
        .standard(Philips)
        .data_format(DataFormat::Data32Channel32)
        .master_clock(true)
        .request_frequency(1)
        .i2s_driver(i2s2);
    rprintln!("actual sample rate is {}", i2s2_driver.sample_rate());
    i2s2_driver.set_rx_interrupt(true);
    i2s2_driver.set_error_interrupt(true);

    let mut i2s3_driver = I2sDriverConfig::new_slave()
        .transmit()
        .standard(Philips)
        .data_format(DataFormat::Data32Channel32)
        .i2s_driver(i2s3);
    i2s3_driver.set_tx_interrupt(true);
    i2s3_driver.set_error_interrupt(true);

    // prepare data to transmit
    for e in FRM_32 {
        i2s3_data_p.enqueue(*e).ok();
    }

    // start drivers
    rprintln!("{} Start i2s2 and i2s3", DWT::cycle_count());
    (
        &mut shared_exti,
        &mut shared_i2s2_driver,
        &mut shared_i2s3_driver,
    )
        .lock(|exti, shared_i2s2_driver, shared_i2s3_driver| {
            i2s2_driver.enable();
            *shared_i2s2_driver = Some(ReceiveDriver::Master(i2s2_driver));
            let ws_pin = i2s3_driver.i2s_peripheral_mut().ws_pin_mut();
            ws_pin.enable_interrupt(exti);
            *shared_i2s3_driver = Some(TransmitDriver::Slave(i2s3_driver));
        });

    //block until test finish 
    while i2s2_data_c.len() < i2s2_data_c.capacity() {}

    //disable driver and release
    let i2s2 = shared_i2s2_driver.lock(|shared_i2s2_driver| {
        if let Some(ReceiveDriver::Master(mut i2s2_driver)) = shared_i2s2_driver.take() {
            i2s2_driver.disable();
            i2s2_driver.release()
        } else {
            panic!()
        }
    });
    let i2s3 = shared_i2s3_driver.lock(|i2s3_driver| {
        if let Some(TransmitDriver::Slave(mut i2s3_driver)) = i2s3_driver.take() {
            i2s3_driver.disable();
            i2s3_driver.release()
        } else { panic!()}
    });

    // get test result
    for e in res_32.iter_mut() {
        *e = i2s2_data_c.dequeue().unwrap_or_default();
    }

    // display result
    for (e, r) in FRM_32.iter().zip(res_32.iter()) {
        let (t, r) = r;
        rprintln!(
            "{:#010x} {:#010x}, {:10} {:#010x} {:#010x}",
            e.0,
            e.1,
            t,
            r.0,
            r.1
        );
    }
    (i2s2,i2s3)
}
