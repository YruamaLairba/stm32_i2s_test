use crate::app::log;
use crate::app::{I2s2, I2s3};
use crate::hal::i2s::stm32_i2s_v12x::driver::*;
use crate::hal::i2s::stm32_i2s_v12x::I2sPeripheral;
use crate::hal::pac::DWT;
use heapless::spsc::*;
use rtt_target::rprintln;

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

pub struct DriverWrap<I> {
    drv: Option<DriverMode<I>>,
    frame_state: FrameState,
    frame:(u32,u32),
}

impl<I: I2sPeripheral> DriverWrap<I> {
    pub fn take(&mut self) -> Option<DriverMode<I>> {
        self.drv.take()
    }

    pub fn replace(&mut self, drv: DriverMode<I>) -> Option<DriverMode<I>> {
        self.drv.replace(drv)
    }

    fn _handle_master_transmit_32bits(
        driver: &mut I2sDriver<I, Master, Transmit, I2sStd>,
        frame_state: &mut FrameState,
        frame: &mut (u32, u32),
        data_c: &mut Consumer<'static, (i32, i32), 8>,
    ) {
        let status = driver.status();
        // it's better to write data first to avoid to trigger udr flag
        if status.txe() {
            let data;
            match (*frame_state, status.chside()) {
                (LeftMsb, Channel::Left) => {
                    let (l, r) = data_c.dequeue().unwrap_or_default();
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

    fn _handle_master_receive_32bits(
        driver: &mut I2sDriver<I, Master, Receive, I2sStd>,
        frame_state: &mut FrameState,
        frame: &mut (u32, u32),
        data_p: &mut Producer<'static, (u32, (i32, i32)), 8>,
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
                    data_p
                        .enqueue((DWT::cycle_count(), (l as i32, r as i32)))
                        .ok();
                    if !data_p.ready() {
                        driver.disable();
                        rprintln!("{} master receive stopped", DWT::cycle_count(),);
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

    pub fn interrupt_handler(&mut self, data_p: &mut Producer<'static, (u32, (i32, i32)), 8>,) {
    }
}
