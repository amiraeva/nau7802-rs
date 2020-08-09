#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Register {
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
pub enum PuCtrlBits {
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
pub enum PgaRegisterBits {
    ChpDis = 0,
    Inv = 3,
    BypassEn,
    OutEn,
    LdoMode,
    RdOtpSel,
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum PgaPwrRegisterBits {
    Curr = 0,
    AdcCurr = 2,
    MstrBiasCurr = 4,
    CapEn = 7,
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Ctrl2RegisterBits {
    CalMod = 0,
    Cals = 2,
    CalError = 3,
    Crs = 4,
    Chs = 7,
}

pub trait RegisterBits {
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

pub enum AfeCalibrationStatus {
    InProgress,
    Failure,
    Success,
}
