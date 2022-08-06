//! Contains test to be done

use crate::app::{I2s2, I2s3};
use heapless::spsc::*;
use rtt_target::{rprint, rprintln};

use crate::hal;

use hal::gpio::ExtiPin;
use hal::i2s::stm32_i2s_v12x::driver::{DataFormat, *};
use hal::i2s::stm32_i2s_v12x::transfer::*;
use hal::pac::DWT;
use hal::pac::EXTI;

use rtic::mutex::prelude::*;

use crate::driver_wrap::*;

use DriverMode::*;

const FRM_32: &[(i16, i16)] = &[
    (0x1111u16 as _, 0x7777u16 as _),
    (0x2222u16 as _, 0x5555u16 as _),
    (0x8888u16 as _, 0xAAAAu16 as _),
    (0xCCCCu16 as _, 0x1000u16 as _),
    (0x3000u16 as _, 0x5000u16 as _),
    (0x7000u16 as _, 0x9000u16 as _),
    (0xB000u16 as _, 0xD000u16 as _),
];

fn slice_contains<T>(slice: &[T], pattern: &[T]) -> bool
where
    T: PartialEq<T>,
{
    if pattern.len() > slice.len() {
        return false;
    }
    for i in 0..=(slice.len() - pattern.len()) {
        if slice[i..(pattern.len() + i)] == *pattern {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slice_contain() {
        let slice = &[0, 1, 2, 3, 4];
        let pat = &[3, 4];
        assert_eq!(slice_contains(slice, pat), true);
    }

    #[test]
    fn slice_not_contain() {
        let slice = &[0, 1, 2, 3, 4];
        let pat = &[1, 4];
        assert_eq!(slice_contains(slice, pat), false);
    }
}

fn check_result<const N: usize>(res: &[(u32, (i16, i16)); N]) {
    let pattern = &FRM_32[1..(FRM_32.len() - 1)];
    let mut cmp = [(0, 0); N];
    for ((_, s), d) in res.iter().zip(cmp.iter_mut()) {
        *d = *s;
    }
    if slice_contains(&cmp, pattern) {
        rprintln!("ok");
    } else {
        rprintln!("failed");
        for (e, r) in FRM_32.iter().zip(res.iter()) {
            let (t, r) = r;
            rprintln!(
                "{:#06x} {:#06x}, {:10} {:#06x} {:#06x}",
                e.0,
                e.1,
                t,
                r.0,
                r.1
            );
        }
    }
}

pub fn master_receive_slave_transmit_driver_interrupt(
    mut shared_exti: &mut impl Mutex<T = EXTI>,
    mut shared_i2s2_driver: &mut impl Mutex<T = DriverWrap<I2s2>>,
    mut shared_i2s3_driver: &mut impl Mutex<T = DriverWrap<I2s3>>,
    i2s2_data_c: &mut Consumer<'static, (u32, (i16, i16)), 8_usize>,
    i2s3_data_p: &mut Producer<'static, (i16, i16), 8_usize>,
    i2s2: I2s2,
    i2s3: I2s3,
) -> (I2s2, I2s3) {
    let mut res = [(0, (0, 0)); 7];

    rprint!("Master Receive + Slave Transmit driver 16 bits with interrupt");

    // Set up drivers
    let mut i2s2_driver = I2sDriverConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(DataFormat::Data16Channel32)
        .master_clock(true)
        .request_frequency(1)
        .i2s_driver(i2s2);
    rprint!(", SR {} ... ", i2s2_driver.sample_rate());
    i2s2_driver.set_rx_interrupt(true);
    i2s2_driver.set_error_interrupt(true);

    let mut i2s3_driver = I2sDriverConfig::new_slave()
        .transmit()
        .standard(I2sStd)
        .data_format(DataFormat::Data16Channel32)
        .i2s_driver(i2s3);
    i2s3_driver.set_tx_interrupt(true);
    i2s3_driver.set_error_interrupt(true);

    // prepare data to transmit
    for e in FRM_32 {
        i2s3_data_p.enqueue(*e).ok();
    }

    // start drivers
    (
        &mut shared_exti,
        &mut shared_i2s2_driver,
        &mut shared_i2s3_driver,
    )
        .lock(|exti, shared_i2s2_driver, shared_i2s3_driver| {
            i2s3_driver.enable();
            //let ws_pin = i2s3_driver.ws_pin_mut();
            //ws_pin.enable_interrupt(exti);
            shared_i2s3_driver.replace(SlaveTransmit16bits(i2s3_driver));
            i2s2_driver.enable();
            shared_i2s2_driver.replace(MasterReceive16bits(i2s2_driver));
        });

    //block until test finish
    while i2s2_data_c.len() < i2s2_data_c.capacity() {}

    //disable driver and release
    let i2s2 = shared_i2s2_driver.lock(|shared_i2s2_driver| {
        if let Some(MasterReceive16bits(mut i2s2_driver)) = shared_i2s2_driver.take() {
            i2s2_driver.disable();
            i2s2_driver.release()
        } else {
            panic!()
        }
    });
    let i2s3 = (&mut shared_i2s3_driver, &mut shared_exti).lock(|i2s3_driver, exti| {
        if let Some(SlaveTransmit16bits(mut i2s3_driver)) = i2s3_driver.take() {
            i2s3_driver.disable();
            let ws_pin = i2s3_driver.ws_pin_mut();
            ws_pin.disable_interrupt(exti);
            i2s3_driver.release()
        } else {
            panic!()
        }
    });

    // get test result
    for e in res.iter_mut() {
        *e = i2s2_data_c.dequeue().unwrap_or_default();
    }

    //flush result queue
    while i2s2_data_c.dequeue().is_some() {}

    // display result
    check_result(&res);
    (i2s2, i2s3)
}

pub fn slave_receive_master_transmit_driver_interrupt(
    mut shared_exti: &mut impl Mutex<T = EXTI>,
    mut shared_i2s2_driver: &mut impl Mutex<T = DriverWrap<I2s2>>,
    mut shared_i2s3_driver: &mut impl Mutex<T = DriverWrap<I2s3>>,
    i2s2_data_c: &mut Consumer<'static, (u32, (i16, i16)), 8_usize>,
    i2s3_data_p: &mut Producer<'static, (i16, i16), 8_usize>,
    i2s2: I2s2,
    i2s3: I2s3,
) -> (I2s2, I2s3) {
    let mut res = [(0, (0, 0)); 7];

    rprint!("Slave Receive + Master Transmit driver 16 bits with interrupt");
    let drv_cfg_base = I2sDriverConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(DataFormat::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    // Set up drivers
    let mut i2s2_driver = drv_cfg_base.to_slave().receive().i2s_driver(i2s2);
    i2s2_driver.set_rx_interrupt(true);
    i2s2_driver.set_error_interrupt(true);

    let mut i2s3_driver = drv_cfg_base.transmit().i2s_driver(i2s3);
    rprint!(", SR {} ... ", i2s3_driver.sample_rate());
    i2s3_driver.set_tx_interrupt(true);

    // prepare data to transmit
    for e in FRM_32 {
        i2s3_data_p.enqueue(*e).ok();
    }

    // start drivers
    (
        &mut shared_exti,
        &mut shared_i2s2_driver,
        &mut shared_i2s3_driver,
    )
        .lock(|exti, shared_i2s2_driver, shared_i2s3_driver| {
            i2s3_driver.enable();
            shared_i2s3_driver.replace(MasterTransmit16bits(i2s3_driver));
            let ws_pin = i2s2_driver.ws_pin_mut();
            ws_pin.enable_interrupt(exti);
            shared_i2s2_driver.replace(SlaveReceive16bits(i2s2_driver));
        });

    //block until test finish
    while i2s2_data_c.len() < i2s2_data_c.capacity() {}

    //disable driver and release
    let i2s2 = (&mut shared_i2s2_driver, &mut shared_exti).lock(|i2s2_driver, exti| {
        if let Some(SlaveReceive16bits(mut i2s2_driver)) = i2s2_driver.take() {
            i2s2_driver.disable();
            let ws_pin = i2s2_driver.ws_pin_mut();
            ws_pin.disable_interrupt(exti);
            i2s2_driver.release()
        } else {
            panic!()
        }
    });
    let i2s3 = shared_i2s3_driver.lock(|i2s3_driver| {
        if let Some(MasterTransmit16bits(mut i2s3_driver)) = i2s3_driver.take() {
            i2s3_driver.disable();
            i2s3_driver.release()
        } else {
            panic!()
        }
    });

    // get test result
    for e in res.iter_mut() {
        *e = i2s2_data_c.dequeue().unwrap_or_default();
    }

    // display result
    check_result(&res);
    (i2s2, i2s3)
}

//#[cfg(FALSE)]
pub fn master_transmit_transfer_block(
    mut shared_exti: &mut impl Mutex<T = EXTI>,
    mut shared_i2s2_driver: &mut impl Mutex<T = DriverWrap<I2s2>>,
    i2s2_data_c: &mut Consumer<'static, (u32, (i16, i16)), 8_usize>,
    i2s2: I2s2,
    i2s3: I2s3,
) -> (I2s2, I2s3) {
    let mut res = [(0, (0, 0)); 7];

    rprint!("Master Transmit Transfer 16 bits block");
    let drv_cfg_base = I2sDriverConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(DataFormat::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    let transfer_cfg_base = I2sTransferConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(marker::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    // Set up drivers and transfert
    let mut i2s2_driver = drv_cfg_base.to_slave().receive().i2s_driver(i2s2);
    i2s2_driver.set_rx_interrupt(true);
    i2s2_driver.set_error_interrupt(true);

    let mut i2s3_transfer = transfer_cfg_base.transmit().i2s_transfer(i2s3);
    rprint!(", SR {} ... ", i2s3_transfer.sample_rate());

    // start drivers
    (&mut shared_exti, &mut shared_i2s2_driver).lock(|exti, shared_i2s2_driver| {
        let ws_pin = i2s2_driver.ws_pin_mut();
        ws_pin.enable_interrupt(exti);
        shared_i2s2_driver.replace(SlaveReceive16bits(i2s2_driver));
    });

    //blocking transmit
    i2s3_transfer.write_iter(FRM_32.iter().copied());

    //block until test finish
    //while i2s2_data_c.len() < i2s2_data_c.capacity() {}

    //disable driver and transfer and release
    let i2s2 = (&mut shared_i2s2_driver, &mut shared_exti).lock(|i2s2_driver, exti| {
        if let Some(SlaveReceive16bits(mut i2s2_driver)) = i2s2_driver.take() {
            i2s2_driver.disable();
            let ws_pin = i2s2_driver.ws_pin_mut();
            ws_pin.disable_interrupt(exti);
            i2s2_driver.release()
        } else {
            panic!()
        }
    });
    let i2s3 = i2s3_transfer.release();

    // get test result
    for e in res.iter_mut() {
        *e = i2s2_data_c.dequeue().unwrap_or_default();
    }

    // display result
    check_result(&res);
    (i2s2, i2s3)
}

//#[cfg(FALSE)]
pub fn master_transmit_transfer_nb(
    mut shared_exti: &mut impl Mutex<T = EXTI>,
    mut shared_i2s2_driver: &mut impl Mutex<T = DriverWrap<I2s2>>,
    i2s2_data_c: &mut Consumer<'static, (u32, (i16, i16)), 8_usize>,
    i2s2: I2s2,
    i2s3: I2s3,
) -> (I2s2, I2s3) {
    let mut res = [(0, (0, 0)); 7];

    rprint!("Master Transmit Transfer 16 bits nb");
    let drv_cfg_base = I2sDriverConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(DataFormat::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    let transfer_cfg_base = I2sTransferConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(marker::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    // Set up drivers and transfert
    let mut i2s2_driver = drv_cfg_base.to_slave().receive().i2s_driver(i2s2);
    i2s2_driver.set_rx_interrupt(true);
    i2s2_driver.set_error_interrupt(true);

    let mut i2s3_transfer = transfer_cfg_base.transmit().i2s_transfer(i2s3);
    rprint!(", SR {} ... ", i2s3_transfer.sample_rate());

    // start drivers
    (&mut shared_exti, &mut shared_i2s2_driver).lock(|exti, shared_i2s2_driver| {
        let ws_pin = i2s2_driver.ws_pin_mut();
        ws_pin.enable_interrupt(exti);
        shared_i2s2_driver.replace(SlaveReceive16bits(i2s2_driver));
    });

    //nb transmit
    for data in FRM_32 {
        while i2s3_transfer.write(*data).is_err() {}
    }
    while i2s3_transfer.write((0, 0)).is_err() {}

    //block until test finish
    //while i2s2_data_c.len() < i2s2_data_c.capacity() {}

    //disable driver and transfer and release
    let i2s2 = (&mut shared_i2s2_driver, &mut shared_exti).lock(|i2s2_driver, exti| {
        if let Some(SlaveReceive16bits(mut i2s2_driver)) = i2s2_driver.take() {
            i2s2_driver.disable();
            let ws_pin = i2s2_driver.ws_pin_mut();
            ws_pin.disable_interrupt(exti);
            i2s2_driver.release()
        } else {
            panic!()
        }
    });
    let i2s3 = i2s3_transfer.release();

    // get test result
    for e in res.iter_mut() {
        *e = i2s2_data_c.dequeue().unwrap_or_default();
    }

    // display result
    check_result(&res);
    (i2s2, i2s3)
}

//#[cfg(FALSE)]
pub fn slave_transmit_transfer_block(
    shared_i2s2_driver: &mut impl Mutex<T = DriverWrap<I2s2>>,
    i2s2_data_c: &mut Consumer<'static, (u32, (i16, i16)), 8_usize>,
    i2s2: I2s2,
    i2s3: I2s3,
) -> (I2s2, I2s3) {
    let mut res = [(0, (0, 0)); 7];

    // erase previous result
    while i2s2_data_c.dequeue().is_some() {}

    rprint!("Slave Transmit Transfer 16 bits block");
    let drv_cfg_base = I2sDriverConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(DataFormat::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    let transfer_cfg_base = I2sTransferConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(marker::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    // Set up drivers and transfert
    let mut i2s2_driver = drv_cfg_base.receive().i2s_driver(i2s2);
    rprint!(", SR {} ... ", i2s2_driver.sample_rate());
    i2s2_driver.set_rx_interrupt(true);
    i2s2_driver.set_error_interrupt(true);

    let mut i2s3_transfer = transfer_cfg_base.to_slave().transmit().i2s_transfer(i2s3);

    // start drivers
    shared_i2s2_driver.lock(|shared_i2s2_driver| {
        i2s2_driver.enable();
        shared_i2s2_driver.replace(MasterReceive16bits(i2s2_driver));
    });

    //blocking transmit
    i2s3_transfer.write_iter(FRM_32[0..7].iter().copied());

    //block until test finish
    //while i2s2_data_c.len() < i2s2_data_c.capacity() {}

    //disable driver and transfer and release
    let i2s2 = shared_i2s2_driver.lock(|i2s2_driver| {
        if let Some(MasterReceive16bits(mut i2s2_driver)) = i2s2_driver.take() {
            i2s2_driver.disable();
            i2s2_driver.release()
        } else {
            panic!()
        }
    });
    let i2s3 = i2s3_transfer.release();

    // get test result
    for e in res.iter_mut() {
        *e = i2s2_data_c.dequeue().unwrap_or_default();
    }

    // display result
    check_result(&res);
    (i2s2, i2s3)
}

//#[cfg(FALSE)]
pub fn slave_transmit_transfer_nb(
    shared_i2s2_driver: &mut impl Mutex<T = DriverWrap<I2s2>>,
    i2s2_data_c: &mut Consumer<'static, (u32, (i16, i16)), 8_usize>,
    i2s2: I2s2,
    i2s3: I2s3,
) -> (I2s2, I2s3) {
    let mut res = [(0, (0, 0)); 7];

    rprint!("Slave Transmit Transfer 16 bits nb");
    let drv_cfg_base = I2sDriverConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(DataFormat::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    let transfer_cfg_base = I2sTransferConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(marker::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    // erase previous result
    while i2s2_data_c.dequeue().is_some() {}

    // Set up drivers and transfert
    let mut i2s2_driver = drv_cfg_base.receive().i2s_driver(i2s2);
    rprint!(", SR {} ... ", i2s2_driver.sample_rate());
    i2s2_driver.set_rx_interrupt(true);
    i2s2_driver.set_error_interrupt(true);

    let mut i2s3_transfer = transfer_cfg_base.to_slave().transmit().i2s_transfer(i2s3);

    // start drivers
    shared_i2s2_driver.lock(|shared_i2s2_driver| {
        i2s2_driver.enable();
        shared_i2s2_driver.replace(MasterReceive16bits(i2s2_driver));
    });

    //nb transmit
    'a: for data in FRM_32.iter() {
        while i2s3_transfer.write(*data).is_err() {
            if i2s2_data_c.len() >= i2s2_data_c.capacity() {
                break 'a;
            }
        }
    }
    // padding
    while i2s3_transfer.write((0xFF, 0xFE)).is_err() {}
    //while i2s3_transfer.write((0xDD,0xCC)).is_err() {}

    //block until test finish

    //disable driver and transfer and release
    let i2s2 = shared_i2s2_driver.lock(|i2s2_driver| {
        if let Some(MasterReceive16bits(mut i2s2_driver)) = i2s2_driver.take() {
            i2s2_driver.disable();
            i2s2_driver.release()
        } else {
            panic!()
        }
    });
    let i2s3 = i2s3_transfer.release();

    // get test result
    for e in res.iter_mut() {
        *e = i2s2_data_c.dequeue().unwrap_or_default();
    }

    // display result
    check_result(&res);
    (i2s2, i2s3)
}

//#[cfg(FALSE)]
pub fn master_receive_transfer_block(
    mut shared_exti: &mut impl Mutex<T = EXTI>,
    mut shared_i2s3_driver: &mut impl Mutex<T = DriverWrap<I2s3>>,
    i2s3_data_p: &mut Producer<'static, (i16, i16), 8_usize>,
    i2s2: I2s2,
    i2s3: I2s3,
) -> (I2s2, I2s3) {
    let mut res = [(0, (0, 0)); 7];

    rprint!("Master Receive Transfer 16 bits block");
    let drv_cfg_base = I2sDriverConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(DataFormat::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    let transfer_cfg_base = I2sTransferConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(marker::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    // Set up drivers and transfer
    let mut i2s2_transfer = transfer_cfg_base.receive().i2s_transfer(i2s2);
    rprint!(", SR {} ... ", i2s2_transfer.sample_rate());

    let mut i2s3_driver = drv_cfg_base.to_slave().transmit().i2s_driver(i2s3);
    i2s3_driver.set_tx_interrupt(true);
    i2s3_driver.set_error_interrupt(true);

    // prepare data to transmit
    for e in FRM_32 {
        i2s3_data_p.enqueue(*e).ok();
    }

    // start drivers
    (&mut shared_i2s3_driver, &mut shared_exti).lock(|shared_i2s3_driver, exti| {
        let ws_pin = i2s3_driver.ws_pin_mut();
        ws_pin.enable_interrupt(exti);
        shared_i2s3_driver.replace(SlaveTransmit16bits(i2s3_driver));
    });

    //blocking transmit
    let mut res_iter = res.iter_mut().peekable();
    i2s2_transfer.read_while(|s| {
        if let Some(r) = res_iter.next() {
            *r = (DWT::cycle_count(), s);
        }
        res_iter.peek().is_some()
    });

    //block until test finish
    //while i2s2_data_c.len() < i2s2_data_c.capacity() {}

    //disable driver and transfer and release
    while i2s3_data_p.len() > 0 {}
    let i2s3 = shared_i2s3_driver.lock(|i2s3_driver| {
        if let Some(SlaveTransmit16bits(mut i2s3_driver)) = i2s3_driver.take() {
            i2s3_driver.disable();
            i2s3_driver.release()
        } else {
            panic!()
        }
    });
    let i2s2 = i2s2_transfer.release();

    // display result
    check_result(&res);
    (i2s2, i2s3)
}

//#[cfg(FALSE)]
pub fn master_receive_transfer_nb(
    mut shared_exti: &mut impl Mutex<T = EXTI>,
    mut shared_i2s3_driver: &mut impl Mutex<T = DriverWrap<I2s3>>,
    i2s3_data_p: &mut Producer<'static, (i16, i16), 8_usize>,
    i2s2: I2s2,
    i2s3: I2s3,
) -> (I2s2, I2s3) {
    let mut res = [(0, (0, 0)); 7];

    rprint!("Master Receive Transfer 16 bits nb");
    let drv_cfg_base = I2sDriverConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(DataFormat::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    let transfer_cfg_base = I2sTransferConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(marker::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    // Set up drivers and transfer
    let mut i2s2_transfer = transfer_cfg_base.receive().i2s_transfer(i2s2);
    rprint!(", SR {} ... ", i2s2_transfer.sample_rate());

    let mut i2s3_driver = drv_cfg_base.to_slave().transmit().i2s_driver(i2s3);
    i2s3_driver.set_tx_interrupt(true);
    i2s3_driver.set_error_interrupt(true);

    // prepare data to transmit
    for e in FRM_32 {
        i2s3_data_p.enqueue(*e).ok();
    }

    // start drivers
    (&mut shared_i2s3_driver, &mut shared_exti).lock(|shared_i2s3_driver, exti| {
        let ws_pin = i2s3_driver.ws_pin_mut();
        ws_pin.enable_interrupt(exti);
        shared_i2s3_driver.replace(SlaveTransmit16bits(i2s3_driver));
    });

    //blocking transmit
    for r in res.iter_mut() {
        let data = loop {
            if let Ok(s) = i2s2_transfer.read() {
                break s;
            }
        };
        *r = (DWT::cycle_count(), data);
    }

    //block until test finish
    //while i2s2_data_c.len() < i2s2_data_c.capacity() {}

    //disable driver and transfer and release
    while i2s3_data_p.len() > 0 {}
    let i2s3 = shared_i2s3_driver.lock(|i2s3_driver| {
        if let Some(SlaveTransmit16bits(mut i2s3_driver)) = i2s3_driver.take() {
            i2s3_driver.disable();
            i2s3_driver.release()
        } else {
            panic!()
        }
    });
    let i2s2 = i2s2_transfer.release();

    // display result
    check_result(&res);
    (i2s2, i2s3)
}

//#[cfg(FALSE)]
pub fn slave_receive_transfer_block(
    shared_i2s3_driver: &mut impl Mutex<T = DriverWrap<I2s3>>,
    i2s3_data_p: &mut Producer<'static, (i16, i16), 8_usize>,
    i2s2: I2s2,
    i2s3: I2s3,
) -> (I2s2, I2s3) {
    let mut res = [(0, (0, 0)); 7];

    rprint!("Slave Receive Transfer 16 bits block");
    let drv_cfg_base = I2sDriverConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(DataFormat::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    let transfer_cfg_base = I2sTransferConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(marker::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    // Set up drivers and transfer
    let mut i2s2_transfer = transfer_cfg_base.to_slave().receive().i2s_transfer(i2s2);

    let mut i2s3_driver = drv_cfg_base.transmit().i2s_driver(i2s3);
    i2s3_driver.set_tx_interrupt(true);
    rprint!(", SR {} ... ", i2s3_driver.sample_rate());

    // prepare data to transmit
    for e in FRM_32 {
        i2s3_data_p.enqueue(*e).ok();
    }

    // start drivers
    shared_i2s3_driver.lock(|shared_i2s3_driver| {
        i2s3_driver.enable();
        shared_i2s3_driver.replace(MasterTransmit16bits(i2s3_driver));
    });

    //blocking transmit
    let mut res_iter = res.iter_mut().peekable();
    i2s2_transfer.read_while(|s| {
        if let Some(r) = res_iter.next() {
            *r = (DWT::cycle_count(), s);
        }
        res_iter.peek().is_some()
    });

    //block until test finish
    //while i2s2_data_c.len() < i2s2_data_c.capacity() {}

    //disable driver and transfer and release
    let i2s3 = shared_i2s3_driver.lock(|i2s3_driver| {
        if let Some(MasterTransmit16bits(mut i2s3_driver)) = i2s3_driver.take() {
            i2s3_driver.disable();
            i2s3_driver.release()
        } else {
            panic!()
        }
    });
    let i2s2 = i2s2_transfer.release();

    // display result
    check_result(&res);
    (i2s2, i2s3)
}

//#[cfg(FALSE)]
pub fn slave_receive_transfer_nb(
    shared_i2s3_driver: &mut impl Mutex<T = DriverWrap<I2s3>>,
    i2s3_data_p: &mut Producer<'static, (i16, i16), 8_usize>,
    i2s2: I2s2,
    i2s3: I2s3,
) -> (I2s2, I2s3) {
    let mut res = [(0, (0, 0)); 7];

    rprint!("Slave Receive Transfer 16 bits nb");
    let drv_cfg_base = I2sDriverConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(DataFormat::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    let transfer_cfg_base = I2sTransferConfig::new_master()
        .receive()
        .standard(I2sStd)
        .data_format(marker::Data16Channel32)
        .master_clock(true)
        .request_frequency(1);

    // Set up drivers and transfer
    let mut i2s2_transfer = transfer_cfg_base.to_slave().receive().i2s_transfer(i2s2);

    let mut i2s3_driver = drv_cfg_base.transmit().i2s_driver(i2s3);
    i2s3_driver.set_tx_interrupt(true);
    rprint!(", SR {} ... ", i2s3_driver.sample_rate());

    // prepare data to transmit
    for e in FRM_32 {
        i2s3_data_p.enqueue(*e).ok();
    }

    // start drivers
    shared_i2s3_driver.lock(|shared_i2s3_driver| {
        i2s3_driver.enable();
        shared_i2s3_driver.replace(MasterTransmit16bits(i2s3_driver));
    });

    //blocking transmit
    for r in res.iter_mut() {
        let data = loop {
            if let Ok(s) = i2s2_transfer.read() {
                break s;
            }
        };
        *r = (DWT::cycle_count(), data);
    }

    //block until test finish
    //while i2s2_data_c.len() < i2s2_data_c.capacity() {}

    //disable driver and transfer and release
    let i2s3 = shared_i2s3_driver.lock(|i2s3_driver| {
        if let Some(MasterTransmit16bits(mut i2s3_driver)) = i2s3_driver.take() {
            i2s3_driver.disable();
            i2s3_driver.release()
        } else {
            panic!()
        }
    });
    let i2s2 = i2s2_transfer.release();

    // display result
    check_result(&res);
    (i2s2, i2s3)
}
