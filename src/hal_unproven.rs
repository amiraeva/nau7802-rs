use crate::Nau7802;

use embedded_hal::{
    adc::{Channel, OneShot},
    blocking::i2c,
};

pub type Channel0 = ();

impl<D: i2c::Read + i2c::Write> OneShot<Self, i32, Channel0> for Nau7802<D> {
    type Error = crate::Error;

    fn read(&mut self, _: &mut Channel0) -> nb::Result<i32, Self::Error> {
        let data_available = self.data_available().map_err(nb::Error::Other)?;

        if !data_available {
            return Err(nb::Error::WouldBlock);
        }

        self.read().map_err(nb::Error::Other)
    }
}

impl<D: i2c::Read + i2c::Write> Channel<Nau7802<D>> for Channel0 {
    type ID = ();
    fn channel() -> Self::ID {
        ()
    }
}
