use crate::Error;
use embedded_hal::i2c;

/// I2C communication interface for BMM350
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    pub(crate) i2c: I2C,
    pub(crate) address: u8,
}

/// Trait for writing data to the BMM350
pub trait WriteData {
    type Error;
    /// Write a single byte to a register
    ///
    /// # Arguments
    ///
    /// * `register` - The register address
    /// * `data` - The byte to write
    fn write_register(&mut self, register: u8, data: u8) -> Result<(), Self::Error>;
    /// Write multiple bytes of data
    ///
    /// # Arguments
    ///
    /// * `payload` - The data to write
    fn write_data(&mut self, payload: &[u8]) -> Result<(), Self::Error>;
}

impl<I2C, E> WriteData for I2cInterface<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    type Error = Error<E>;
    fn write_register(&mut self, register: u8, data: u8) -> Result<(), Self::Error> {
        let payload: [u8; 2] = [register, data];
        self.i2c.write(self.address, &payload).map_err(Error::Comm)
    }

    fn write_data(&mut self, payload: &[u8]) -> Result<(), Self::Error> {
        self.i2c.write(self.address, payload).map_err(Error::Comm)
    }
}

pub trait ReadData {
    type Error;
    /// Read a single byte from a register
    ///
    /// # Arguments
    ///
    /// * `register` - The register address to read from
    fn read_register(&mut self, register: u8) -> Result<u8, Self::Error>;
    /// Read multiple bytes of data
    ///
    /// # Arguments
    ///
    /// * `payload` - Buffer to store the read data
    fn read_data<'a>(&mut self, payload: &'a mut [u8]) -> Result<&'a [u8], Self::Error>;
}

impl<I2C, E> ReadData for I2cInterface<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    type Error = Error<E>;
    fn read_register(&mut self, register: u8) -> Result<u8, Self::Error> {
        let mut temp_data = [0u8; 3];
        self.i2c
            .write_read(self.address, &[register], &mut temp_data)
            .map_err(Error::Comm)?;
        Ok(temp_data[2])
    }

    fn read_data<'a>(&mut self, payload: &'a mut [u8]) -> Result<&'a [u8], Error<E>> {
        let address = payload[0];
        let len = payload.len();
        let data = &mut payload[1..len];

        let total_len = data.len() + 2;
        let mut temp_buf = [0u8; 128]; // Temporary buffer to hold dummy bytes and data

        self.i2c
            .write_read(self.address, &[address], &mut temp_buf[..total_len])
            .map_err(Error::Comm)?;

        // Copy data from temp_buf to data, skipping dummy bytes
        data.copy_from_slice(&temp_buf[2..total_len]);

        Ok(data)
    }
}
