use crate::app::log;
use crate::app::{I2s2, I2s3};
use crate::hal::gpio::ExtiPin;
use crate::hal::i2s::stm32_i2s_v12x::driver::*;
use crate::hal::i2s::stm32_i2s_v12x::I2sPeripheral;
use crate::hal::pac::DWT;
use crate::hal::pac::EXTI;
use heapless::spsc::*;
use rtic::mutex::prelude::*;

type I2sStd = Philips;

// Part of the frame we currently transmit or receive
#[derive(Copy, Clone)]
enum FrameState {
    LeftMsb,
    LeftLsb,
    RightMsb,
    RightLsb,
}
use FrameState::*;

pub enum DriverMode<I> {
    SlaveTransmit16bits(I2sDriver<I, Slave, Transmit, I2sStd>),
    MasterTransmit16bits(I2sDriver<I, Master, Transmit, I2sStd>),
    SlaveReceive16bits(I2sDriver<I, Slave, Receive, I2sStd>),
    MasterReceive16bits(I2sDriver<I, Master, Receive, I2sStd>),
    SlaveTransmit32bits(I2sDriver<I, Slave, Transmit, I2sStd>),
    MasterTransmit32bits(I2sDriver<I, Master, Transmit, I2sStd>),
    SlaveReceive32bits(I2sDriver<I, Slave, Receive, I2sStd>),
    MasterReceive32bits(I2sDriver<I, Master, Receive, I2sStd>),
}
use DriverMode::*;

pub struct DriverWrap<I> {
    pub drv: Option<DriverMode<I>>,
    frame_state: FrameState,
    frame: (u32, u32),
}

fn _slave_transmit_16bits_interrupt(
    driver: &mut I2sDriver<I2s3, Slave, Transmit, I2sStd>,
    exti: &mut impl Mutex<T = EXTI>,
    frame_state: &mut FrameState,
    frame: &mut (u32, u32),
    data_16_c: &mut Consumer<'static, (i16, i16), 8>,
) {
    let status = driver.status();
    // it's better to write data first to avoid to trigger udr flag
    if status.txe() {
        let data;
        match (*frame_state, status.chside()) {
            (LeftMsb, Channel::Left) => {
                let (l, r) = data_16_c.dequeue().unwrap_or_default();
                *frame = (l as u32, r as u32);
                data = (frame.0 & 0xFFFF) as u16;
                *frame_state = RightMsb;
            }
            (RightMsb, Channel::Right) => {
                data = (frame.1 & 0xFFFF) as u16;
                *frame_state = LeftMsb;
            }
            // in case of udr this resynchronize tracked and actual channel
            _ => {
                *frame_state = LeftMsb;
                data = 0; //garbage data to avoid additional underrrun
            }
        }
        driver.write_data_register(data);
    }
    if status.fre() {
        log::spawn(DWT::cycle_count(), "i2s3 Frame error").ok();
        driver.disable();
        exti.lock(|exti| {
            driver
                .i2s_peripheral_mut()
                .ws_pin_mut()
                .enable_interrupt(exti)
        })
    }
    if status.udr() {
        log::spawn(DWT::cycle_count(), "i2s3 udr").ok();
        driver.status();
        driver.write_data_register(0);
    }
}

fn _slave_transmit_32bits_interrupt(
    driver: &mut I2sDriver<I2s3, Slave, Transmit, I2sStd>,
    exti: &mut impl Mutex<T = EXTI>,
    frame_state: &mut FrameState,
    frame: &mut (u32, u32),
    data_32_c: &mut Consumer<'static, (i32, i32), 8>,
) {
    let status = driver.status();
    // it's better to write data first to avoid to trigger udr flag
    if status.txe() {
        let data;
        match (*frame_state, status.chside()) {
            (LeftMsb, Channel::Left) => {
                let (l, r) = data_32_c.dequeue().unwrap_or_default();
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
        driver.write_data_register(data);
    }
    if status.fre() {
        log::spawn(DWT::cycle_count(), "i2s3 Frame error").ok();
        driver.disable();
        exti.lock(|exti| {
            driver
                .i2s_peripheral_mut()
                .ws_pin_mut()
                .enable_interrupt(exti)
        })
    }
    if status.udr() {
        log::spawn(DWT::cycle_count(), "i2s3 udr").ok();
        driver.status();
        driver.write_data_register(0);
    }
}

fn _master_transmit_16bits_interrupt<I: I2sPeripheral>(
    driver: &mut I2sDriver<I, Master, Transmit, I2sStd>,
    frame_state: &mut FrameState,
    frame: &mut (u32, u32),
    data_16_c: &mut Consumer<'static, (i16, i16), 8>,
) {
    let status = driver.status();
    // it's better to write data first to avoid to trigger udr flag
    if status.txe() {
        let data;
        match (*frame_state, status.chside()) {
            (LeftMsb, Channel::Left) => {
                let (l, r) = data_16_c.dequeue().unwrap_or_default();
                *frame = (l as u32, r as u32);
                data = (frame.0 & 0xFFFF) as u16;
                *frame_state = RightMsb;
            }
            (RightMsb, Channel::Right) => {
                data = (frame.1 & 0xFFFF) as u16;
                *frame_state = LeftMsb;
            }
            // in case of udr this resynchronize tracked and actual channel
            _ => {
                *frame_state = LeftMsb;
                data = 0; //garbage data to avoid additional underrrun
            }
        }
        driver.write_data_register(data);
    }
}

fn _master_transmit_32bits_interrupt<I: I2sPeripheral>(
    driver: &mut I2sDriver<I, Master, Transmit, I2sStd>,
    frame_state: &mut FrameState,
    frame: &mut (u32, u32),
    data_32_c: &mut Consumer<'static, (i32, i32), 8>,
) {
    let status = driver.status();
    // it's better to write data first to avoid to trigger udr flag
    if status.txe() {
        let data;
        match (*frame_state, status.chside()) {
            (LeftMsb, Channel::Left) => {
                let (l, r) = data_32_c.dequeue().unwrap_or_default();
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
        driver.write_data_register(data);
    }
}

fn _slave_receive_16bits_interrupt(
    driver: &mut I2sDriver<I2s2, Slave, Receive, I2sStd>,
    exti: &mut impl Mutex<T = EXTI>,
    frame_state: &mut FrameState,
    frame: &mut (u32, u32),
    data_16_p: &mut Producer<'static, (u32, (i16, i16)), 8>,
) {
    let status = driver.status();
    // It's better to read first to avoid triggering ovr flag
    if status.rxne() {
        let data = driver.read_data_register();
        match (*frame_state, status.chside()) {
            (LeftMsb, Channel::Left) => {
                frame.0 |= data as u32;
                *frame_state = RightMsb;
            }
            (RightMsb, Channel::Right) => {
                frame.1 |= data as u32;
                // defer sample processing to another task
                let (l, r) = *frame;
                data_16_p
                    .enqueue((DWT::cycle_count(), (l as i16, r as i16)))
                    .ok();
                if !data_16_p.ready() {
                    driver.disable();
                }
                *frame_state = LeftMsb;
            }
            // in case of ovr this resynchronize at start of new frame
            _ => {
                log::spawn(DWT::cycle_count(), "Slave Receive Channel Err").ok();
                *frame_state = LeftMsb;
            }
        }
    }
    if status.fre() {
        log::spawn(DWT::cycle_count(), "i2s3 Frame error").ok();
        driver.disable();
        exti.lock(|exti| {
            driver
                .i2s_peripheral_mut()
                .ws_pin_mut()
                .enable_interrupt(exti)
        })
    }
    if status.ovr() {
        log::spawn(DWT::cycle_count(), "i2s2 Overrun").ok();
        // sequence to delete ovr flag
        driver.read_data_register();
        driver.status();
    }
}

fn _slave_receive_32bits_interrupt(
    driver: &mut I2sDriver<I2s2, Slave, Receive, I2sStd>,
    exti: &mut impl Mutex<T = EXTI>,
    frame_state: &mut FrameState,
    frame: &mut (u32, u32),
    data_32_p: &mut Producer<'static, (u32, (i32, i32)), 8>,
) {
    let status = driver.status();
    // It's better to read first to avoid triggering ovr flag
    if status.rxne() {
        let data = driver.read_data_register();
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
                data_32_p
                    .enqueue((DWT::cycle_count(), (l as i32, r as i32)))
                    .ok();
                if !data_32_p.ready() {
                    driver.disable();
                }
                *frame_state = LeftMsb;
            }
            // in case of ovr this resynchronize at start of new frame
            _ => {
                log::spawn(DWT::cycle_count(), "Slave Receive Channel Err").ok();
                *frame_state = LeftMsb;
            }
        }
    }
    if status.fre() {
        log::spawn(DWT::cycle_count(), "i2s3 Frame error").ok();
        driver.disable();
        exti.lock(|exti| {
            driver
                .i2s_peripheral_mut()
                .ws_pin_mut()
                .enable_interrupt(exti)
        })
    }
    if status.ovr() {
        log::spawn(DWT::cycle_count(), "i2s2 Overrun").ok();
        // sequence to delete ovr flag
        driver.read_data_register();
        driver.status();
    }
}

fn _master_receive_16bits_interrupt<I: I2sPeripheral>(
    driver: &mut I2sDriver<I, Master, Receive, I2sStd>,
    frame_state: &mut FrameState,
    frame: &mut (u32, u32),
    data_16_p: &mut Producer<'static, (u32, (i16, i16)), 8>,
) {
    let status = driver.status();
    // It's better to read first to avoid triggering ovr flag
    if status.rxne() {
        let data = driver.read_data_register();
        match (*frame_state, status.chside()) {
            (LeftMsb, Channel::Left) => {
                frame.0 |= data as u32;
                *frame_state = RightMsb;
            }
            (RightMsb, Channel::Right) => {
                frame.1 |= data as u32;
                // defer sample processing to another task
                let (l, r) = *frame;
                data_16_p
                    .enqueue((DWT::cycle_count(), (l as i16, r as i16)))
                    .ok();
                if !data_16_p.ready() {
                    driver.disable();
                }
                *frame_state = LeftMsb;
            }
            // in case of ovr this resynchronize at start of new frame
            _ => {
                log::spawn(DWT::cycle_count(), "Master Receive Channel Err").ok();
                *frame_state = LeftMsb;
            }
        }
    }
    if status.ovr() {
        log::spawn(DWT::cycle_count(), "Master Receive Overrun").ok();
        // sequence to delete ovr flag
        driver.read_data_register();
        driver.status();
    }
}


fn _master_receive_32bits_interrupt<I: I2sPeripheral>(
    driver: &mut I2sDriver<I, Master, Receive, I2sStd>,
    frame_state: &mut FrameState,
    frame: &mut (u32, u32),
    data_32_p: &mut Producer<'static, (u32, (i32, i32)), 8>,
) {
    let status = driver.status();
    // It's better to read first to avoid triggering ovr flag
    if status.rxne() {
        let data = driver.read_data_register();
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
                data_32_p
                    .enqueue((DWT::cycle_count(), (l as i32, r as i32)))
                    .ok();
                if !data_32_p.ready() {
                    driver.disable();
                }
                *frame_state = LeftMsb;
            }
            // in case of ovr this resynchronize at start of new frame
            _ => {
                log::spawn(DWT::cycle_count(), "Master Receive Channel Err").ok();
                *frame_state = LeftMsb;
            }
        }
    }
    if status.ovr() {
        log::spawn(DWT::cycle_count(), "Master Receive Overrun").ok();
        // sequence to delete ovr flag
        driver.read_data_register();
        driver.status();
    }
}

impl<I: I2sPeripheral> DriverWrap<I> {
    pub fn new(drv: Option<DriverMode<I>>) -> Self {
        Self {
            drv,
            frame_state: LeftMsb,
            frame: (0, 0),
        }
    }

    pub fn take(&mut self) -> Option<DriverMode<I>> {
        self.frame_state = LeftMsb;
        self.frame = (0,0);
        self.drv.take()
    }

    pub fn replace(&mut self, drv: DriverMode<I>) -> Option<DriverMode<I>> {
        self.drv.replace(drv)
    }

    //reset frame tracking
    pub fn reset_frame(&mut self) {
        self.frame_state = LeftMsb;
        self.frame = (0,0);
    }
}

impl DriverWrap<I2s3> {
    pub fn transmit_interrupt_handler(
        &mut self,
        exti: &mut impl Mutex<T = EXTI>,
        data_16_c: &mut Consumer<'static, (i16, i16), 8>,
        data_32_c: &mut Consumer<'static, (i32, i32), 8>,
    ) {
        match self.drv {
            Some(SlaveTransmit16bits(ref mut drv)) => _slave_transmit_16bits_interrupt(
                drv,
                exti,
                &mut self.frame_state,
                &mut self.frame,
                data_16_c,
            ),
            Some(SlaveTransmit32bits(ref mut drv)) => _slave_transmit_32bits_interrupt(
                drv,
                exti,
                &mut self.frame_state,
                &mut self.frame,
                data_32_c,
            ),
            Some(MasterTransmit16bits(ref mut drv)) => _master_transmit_16bits_interrupt(
                drv,
                &mut self.frame_state,
                &mut self.frame,
                data_16_c,
            ),
            Some(MasterTransmit32bits(ref mut drv)) => _master_transmit_32bits_interrupt(
                drv,
                &mut self.frame_state,
                &mut self.frame,
                data_32_c,
            ),
            _ => unimplemented!(),
        }
    }
}

impl DriverWrap<I2s2> {
    pub fn receive_interrupt_handler(
        &mut self,
        exti: &mut impl Mutex<T = EXTI>,
        data_16_p: &mut Producer<'static, (u32, (i16, i16)), 8>,
        data_32_p: &mut Producer<'static, (u32, (i32, i32)), 8>,
    ) {
        match self.drv {
            Some(SlaveReceive16bits(ref mut drv)) => _slave_receive_16bits_interrupt(
                drv,
                exti,
                &mut self.frame_state,
                &mut self.frame,
                data_16_p,
            ),
            Some(SlaveReceive32bits(ref mut drv)) => _slave_receive_32bits_interrupt(
                drv,
                exti,
                &mut self.frame_state,
                &mut self.frame,
                data_32_p,
            ),
            Some(MasterReceive16bits(ref mut drv)) => _master_receive_16bits_interrupt(
                drv,
                &mut self.frame_state,
                &mut self.frame,
                data_16_p,
            ),
            Some(MasterReceive32bits(ref mut drv)) => _master_receive_32bits_interrupt(
                drv,
                &mut self.frame_state,
                &mut self.frame,
                data_32_p,
            ),
            _ => unimplemented!(),
        }
    }
}
