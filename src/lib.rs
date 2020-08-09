#![no_std]

use byteorder::ByteOrder as _;
use core::{iter, slice};
use embedded_hal::blocking::i2c;

pub struct Nau7802<D: i2c::Read + i2c::Write> {
    i2c_dev: D,
}

impl<D: i2c::Read + i2c::Write> Nau7802<D> {
    const DEVICE_ADDRESS: u8 = 0x2A;

    #[inline]
    pub fn new(i2c_dev: D) -> Result<Self> {
        Self::new_with_settings(i2c_dev, Ldo::L3v3, Gain::G128, SamplesPerSecond::SPS320)
    }

    pub fn new_with_settings(
        i2c_dev: D,
        ldo: Ldo,
        gain: Gain,
        sps: SamplesPerSecond,
    ) -> Result<Self> {
        let mut adc = Self { i2c_dev };

        adc.start_reset()?;
        // need 1 ms delay here maybe?
        adc.finish_reset()?;
        adc.power_up()?;
        adc.set_ldo(ldo)?;
        adc.set_gain(gain)?;
        adc.set_sample_rate(sps)?;
        adc.misc_init()?;
        adc.begin_afe_calibration()?;

        Ok(adc)
    }

    pub fn data_available(&mut self) -> Result<bool> {
        self.get_bit(Register::PuCtrl, PuCtrlBits::CR)
    }

    // assumes that data_avaiable has been called and returned true
    pub fn read(&mut self) -> Result<i32> {
        self.request_register(Register::AdcoB2)?;

        let mut buf = [0u8; 3]; // will hold an i24
        self.i2c_dev
            .read(Self::DEVICE_ADDRESS, &mut buf)
            .map_err(|_| Error::I2cError)?;

        let adc_result = byteorder::BigEndian::read_i24(&buf);
        Ok(adc_result)
    }

    fn begin_afe_calibration(&mut self) -> Result<()> {
        self.set_bit(Register::Ctrl2, Ctrl2RegisterBits::Cals)
    }

    fn poll_afe_calibration_status(&mut self) -> Result<AfeCalibrationStatus> {
        if self.get_bit(Register::Ctrl2, Ctrl2RegisterBits::Cals)? {
            return Ok(AfeCalibrationStatus::InProgress);
        }

        if self.get_bit(Register::Ctrl2, Ctrl2RegisterBits::CalError)? {
            return Ok(AfeCalibrationStatus::Failure);
        }

        Ok(AfeCalibrationStatus::Success)
    }

    fn set_sample_rate(&mut self, sps: SamplesPerSecond) -> Result<()> {
        const SPS_MASK: u8 = 0b10001111;
        const SPS_START_BIT_IDX: u8 = 4;

        self.set_function_helper(Register::Ctrl2, SPS_MASK, SPS_START_BIT_IDX, sps as _)
    }

    fn set_gain(&mut self, gain: Gain) -> Result<()> {
        const GAIN_MASK: u8 = 0b11111000;
        const GAIN_START_BIT: u8 = 0;
        // let mut val = self.get_register(Register::Ctrl1)?;

        // val &= GAIN_MASK;
        // val |= gain as u8;

        // self.set_register(Register::Ctrl1, val)
        self.set_function_helper(Register::Ctrl1, GAIN_MASK, GAIN_START_BIT, gain as _)
    }

    fn set_ldo(&mut self, ldo: Ldo) -> Result<()> {
        const LDO_MASK: u8 = 0b11000111;
        const LDO_START_BIT: u8 = 3;

        // let mut val = self.get_register(Register::Ctrl1)?;
        // val &= LDO_MASK;
        // val |= (ldo as u8) << LDO_START_BIT_IDX;

        // self.set_register(Register::Ctrl1, val)?;

        self.set_function_helper(Register::Ctrl1, LDO_MASK, LDO_START_BIT, ldo as _)?;

        self.set_bit(Register::PuCtrl, PuCtrlBits::AVDDS)
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

    fn power_up(&mut self) -> Result<()> {
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

    fn start_reset(&mut self) -> Result<()> {
        self.set_bit(Register::PuCtrl, PuCtrlBits::RR)
    }

    fn finish_reset(&mut self) -> Result<()> {
        self.clear_bit(Register::PuCtrl, PuCtrlBits::RR)
    }

    fn misc_init(&mut self) -> Result<()> {
        const TURN_OFF_CLK_CHPL: u8 = 0x30;

        // Turn off CLK_CHP. From 9.1 power on sequencing
        self.set_register(Register::Adc, TURN_OFF_CLK_CHPL)?;

        // Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note
        self.set_bit(Register::PgaPwr, PgaPwrRegisterBits::CapEn)
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

#[cfg(feature = "embedded-hal-adc")]
mod unproven {
    use embedded_hal::adc::{Channel, OneShot};

    impl<D: i2c::Read + i2c::Write> OneShot<Self, i32, Channel0> for Nau7802<D> {
        type Error = Error;

        fn read(&mut self, _: &mut Channel0) -> nb::Result<i32, Self::Error> {
            let data_available = self.data_available().map_err(nb::Error::Other)?;

            if !data_available {
                return Err(nb::Error::WouldBlock);
            }

            self.read().map_err(nb::Error::Other)
        }
    }

    type Channel0 = ();

    impl<D: i2c::Read + i2c::Write> Channel<Nau7802<D>> for Channel0 {
        type ID = ();
        fn channel() -> Self::ID {
            ()
        }
    }
}

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    I2cError,
    PowerupFailed,
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum Register {
    PuCtrl = 0x00,
    Ctrl1,
    Ctrl2,
    Ocal1B2,
    Ocal1B1,
    Ocal1B0,
    Gcal1B3,
    Gcal1B2,
    Gcal1B1,
    Gcal1B0,
    Ocal2B2,
    Ocal2B1,
    Ocal2B0,
    Gcal2B3,
    Gcal2B2,
    Gcal2B1,
    Gcal2B0,
    I2CControl,
    AdcoB2,
    AdcoB1,
    AdcoB0,
    Adc = 0x15, //Shared ADC and OTP 32:24
    OtpB1,      //OTP 23:16 or 7:0?
    OtpB0,      //OTP 15:8
    Pga = 0x1B,
    PgaPwr = 0x1C,
    DeviceRev = 0x1F,
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum PuCtrlBits {
    RR = 0,
    PUD,
    PUA,
    PUR,
    CS,
    CR,
    OSCS,
    AVDDS,
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum PgaRegisterBits {
    ChpDis = 0,
    Inv = 3,
    BypassEn,
    OutEn,
    LdoMode,
    RdOtpSel,
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum PgaPwrRegisterBits {
    Curr = 0,
    AdcCurr = 2,
    MstrBiasCurr = 4,
    CapEn = 7,
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum Ctrl2RegisterBits {
    CalMod = 0,
    Cals = 2,
    CalError = 3,
    Crs = 4,
    Chs = 7,
}

trait RegisterBits {
    fn get(&self) -> u8;
}

macro_rules! impl_register_bits {
    ($($type:ident),*) => {
        $(
            impl RegisterBits for $type {
                fn get(&self) -> u8 {
                    *self as _
                }
            }
        )*
    }
}

impl_register_bits!(
    PuCtrlBits,
    PgaRegisterBits,
    PgaPwrRegisterBits,
    Ctrl2RegisterBits
);

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Ldo {
    L2v4 = 0b111,
    L2v7 = 0b110,
    L3v0 = 0b101,
    L3v3 = 0b100,
    L3v6 = 0b011,
    L3v9 = 0b010,
    L3v2 = 0b001,
    L4v5 = 0b000,
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Gain {
    G128 = 0b111,
    G64 = 0b110,
    G32 = 0b101,
    G16 = 0b100,
    G8 = 0b011,
    G4 = 0b010,
    G2 = 0b001,
    G1 = 0b000,
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum SamplesPerSecond {
    SPS320 = 0b111,
    SPS80 = 0b011,
    SPS40 = 0b010,
    SPS20 = 0b001,
    SPS10 = 0b000,
}

enum AfeCalibrationStatus {
    InProgress,
    Failure,
    Success,
}
