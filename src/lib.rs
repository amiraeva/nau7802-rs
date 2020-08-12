#![no_std]

use byteorder::ByteOrder as _;
use core::{iter, slice};
use embedded_hal::blocking::{i2c, delay::DelayMs};

#[cfg(feature = "embedded-hal-adc")]
mod hal_unproven;

#[cfg(feature = "embedded-hal-adc")]
pub use hal_unproven::*;

mod constants;
use constants::*;

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    I2cError,
    PowerupFailed,
}

pub struct Nau7802<D: i2c::Read + i2c::Write> {
    i2c_dev: D,
}

impl<D: i2c::Read + i2c::Write> Nau7802<D> {
    const DEVICE_ADDRESS: u8 = 0x2A;

    #[inline]
    pub fn new<T: From<u8>, W: DelayMs<T>>(i2c_dev: D, wait: &mut W) -> Result<Self> {
        Self::new_with_settings(i2c_dev, Ldo::L3v3, Gain::G128, SamplesPerSecond::SPS320, wait)
    }

    pub fn new_with_settings<T: From<u8>, W: DelayMs<T>>(
        i2c_dev: D,
        ldo: Ldo,
        gain: Gain,
        sps: SamplesPerSecond,
        wait: &mut W
    ) -> Result<Self> {
        let mut adc = Self { i2c_dev };

        adc.start_reset()?;
        // need 1 ms delay here maybe?
        wait.delay_ms(T::from(1));
        adc.finish_reset()?;
        adc.power_up()?;
        adc.set_ldo(ldo)?;
        adc.set_gain(gain)?;
        adc.set_sample_rate(sps)?;
        adc.misc_init()?;
        adc.begin_afe_calibration()?;
        wait.delay_ms(T::from(1));

        Ok(adc)
    }

    pub fn destroy(self) -> D {
        let Self { i2c_dev } = self;
        i2c_dev
    }

    pub fn data_available(&mut self) -> Result<bool> {
        self.get_bit(Register::PuCtrl, PuCtrlBits::CR)
    }

    /// Checks for new data, returns nb::Error::WouldBlock if unavailable
    pub fn read(&mut self) -> nb::Result<i32, Error> {
        let data_available = self.data_available().map_err(nb::Error::Other)?;

        if !data_available {
            return Err(nb::Error::WouldBlock);
        }

        self.read_unchecked().map_err(nb::Error::Other)
    }

    /// assumes that data_avaiable has been called and returned true
    pub fn read_unchecked(&mut self) -> Result<i32> {
        self.request_register(Register::AdcoB2)?;

        let mut buf = [0u8; 3]; // will hold an i24
        self.i2c_dev
            .read(Self::DEVICE_ADDRESS, &mut buf)
            .map_err(|_| Error::I2cError)?;

        let adc_result = byteorder::BigEndian::read_i24(&buf);
        Ok(adc_result)
    }

    pub fn begin_afe_calibration(&mut self) -> Result<()> {
        self.set_bit(Register::Ctrl2, Ctrl2RegisterBits::Cals)
    }

    pub fn poll_afe_calibration_status(&mut self) -> Result<AfeCalibrationStatus> {
        if self.get_bit(Register::Ctrl2, Ctrl2RegisterBits::Cals)? {
            return Ok(AfeCalibrationStatus::InProgress);
        }

        if self.get_bit(Register::Ctrl2, Ctrl2RegisterBits::CalError)? {
            return Ok(AfeCalibrationStatus::Failure);
        }

        Ok(AfeCalibrationStatus::Success)
    }

    pub fn set_sample_rate(&mut self, sps: SamplesPerSecond) -> Result<()> {
        const SPS_MASK: u8 = 0b10001111;
        const SPS_START_BIT_IDX: u8 = 4;

        self.set_function_helper(Register::Ctrl2, SPS_MASK, SPS_START_BIT_IDX, sps as _)
    }

    pub fn set_gain(&mut self, gain: Gain) -> Result<()> {
        const GAIN_MASK: u8 = 0b11111000;
        const GAIN_START_BIT: u8 = 0;

        self.set_function_helper(Register::Ctrl1, GAIN_MASK, GAIN_START_BIT, gain as _)
    }

    pub fn set_ldo(&mut self, ldo: Ldo) -> Result<()> {
        const LDO_MASK: u8 = 0b11000111;
        const LDO_START_BIT: u8 = 3;

        self.set_function_helper(Register::Ctrl1, LDO_MASK, LDO_START_BIT, ldo as _)?;

        self.set_bit(Register::PuCtrl, PuCtrlBits::AVDDS)
    }

    pub fn power_up(&mut self) -> Result<()> {
        const NUM_ATTEMPTS: usize = 100;

        self.set_bit(Register::PuCtrl, PuCtrlBits::PUD)?;
        self.set_bit(Register::PuCtrl, PuCtrlBits::PUA)?;

        let check_powered_up = || self.get_bit(Register::PuCtrl, PuCtrlBits::PUR);

        let powered_up = iter::repeat_with(check_powered_up)
            .take(NUM_ATTEMPTS)
            .filter_map(Result::ok)
            .any(|rdy| rdy == true);

        if powered_up {
            Ok(())
        } else {
            Err(Error::PowerupFailed)
        }
    }

    pub fn start_reset(&mut self) -> Result<()> {
        self.set_bit(Register::PuCtrl, PuCtrlBits::RR)
    }

    pub fn finish_reset(&mut self) -> Result<()> {
        self.clear_bit(Register::PuCtrl, PuCtrlBits::RR)
    }

    pub fn misc_init(&mut self) -> Result<()> {
        const TURN_OFF_CLK_CHPL: u8 = 0x30;

        // Turn off CLK_CHP. From 9.1 power on sequencing
        self.set_register(Register::Adc, TURN_OFF_CLK_CHPL)?;

        // Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note
        self.set_bit(Register::PgaPwr, PgaPwrRegisterBits::CapEn)
    }

    fn set_function_helper(
        &mut self,
        reg: Register,
        mask: u8,
        start_idx: u8,
        new_val: u8,
    ) -> Result<()> {
        let mut val = self.get_register(reg)?;
        val &= mask;
        val |= new_val << start_idx;

        self.set_register(reg, val)
    }

    fn set_bit<B: RegisterBits>(&mut self, addr: Register, bit_idx: B) -> Result<()> {
        let mut val = self.get_register(addr)?;
        val |= 1 << bit_idx.get();
        self.set_register(addr, val)
    }

    fn clear_bit<B: RegisterBits>(&mut self, addr: Register, bit_idx: B) -> Result<()> {
        let mut val = self.get_register(addr)?;
        val &= !(1 << bit_idx.get());
        self.set_register(addr, val)
    }

    fn get_bit<B: RegisterBits>(&mut self, addr: Register, bit_idx: B) -> Result<bool> {
        let mut val = self.get_register(addr)?;
        val &= 1 << bit_idx.get();
        Ok(val != 0)
    }

    fn set_register(&mut self, reg: Register, val: u8) -> Result<()> {
        let transaction = [reg as _, val];

        self.i2c_dev
            .write(Self::DEVICE_ADDRESS, &transaction)
            .map_err(|_| Error::I2cError)
    }

    fn get_register(&mut self, reg: Register) -> Result<u8> {
        self.request_register(reg)?;

        let mut val = 0;
        self.i2c_dev
            .read(Self::DEVICE_ADDRESS, slice::from_mut(&mut val))
            .map_err(|_| Error::I2cError)?;

        Ok(val)
    }

    fn request_register(&mut self, reg: Register) -> Result<()> {
        let reg = reg as u8;

        self.i2c_dev
            .write(Self::DEVICE_ADDRESS, slice::from_ref(&reg))
            .map_err(|_| Error::I2cError)
    }
}
