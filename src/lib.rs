#![no_std]

/// BMM350 driver for Rust
///
/// This module provides a high-level interface for interacting with the Bosch BMM350 magnetometer.
/// It supports both I2C interfaces and allows for configuration of magnetometer settings.
pub mod device;
pub mod interface;
mod registers;
pub use registers::Register;
mod types;
pub use types::{
    AverageNum, AxisEnableDisable, Bandwidth, CtrlUser, DataRate, Error, I2cWdtEnable,
    I2cWdtSelect, InterruptDrive, InterruptEnableDisable, InterruptLatch, InterruptMap,
    InterruptPolarity, MagCompensation, PerformanceMode, PowerMode, SelfTestMode, Sensor3DData,
    Sensor3DDataScaled,
};
mod sensor_data;
pub use sensor_data::*;

/// Main struct representing the BMM350 device
pub struct Bmm350<DI, D> {
    /// Communication interface (I2C or SPI)
    iface: DI,
    /// Delay provider
    delay: D,
    /// Current magnetometer range
    mag_range: f32,
    /// Variant ID
    var_id: u8,
    /// Magnetometer compensation data
    mag_comp: MagCompensation,
}

/// Configuration for the magnetometer
#[derive(Debug, Clone, Copy)]
pub struct MagConfig {
    /// Output data rate
    pub odr: DataRate,
    /// Performance mode (averaging)
    pub performance: PerformanceMode,
    /// Bandwidth
    pub bw: Bandwidth,
    /// Power mode
    pub mode: PowerMode,
}

impl MagConfig {
    /// Create a new MagConfigBuilder
    pub fn builder() -> MagConfigBuilder {
        MagConfigBuilder::default()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct MagConfigBuilder {
    odr: Option<DataRate>,
    performance: Option<PerformanceMode>,
    bw: Option<Bandwidth>,
    mode: Option<PowerMode>,
}

/// Builder for MagConfig
impl Default for MagConfigBuilder {
    fn default() -> Self {
        Self {
            odr: None,
            performance: None,
            bw: None,
            mode: None,
        }
    }
}

impl MagConfigBuilder {
    /// Set the output data rate
    pub fn odr(mut self, odr: DataRate) -> Self {
        self.odr = Some(odr);
        self
    }

    /// Set the performance mode
    pub fn performance(mut self, performance: PerformanceMode) -> Self {
        self.performance = Some(performance);
        self
    }

    /// Set the bandwidth
    pub fn bw(mut self, bw: Bandwidth) -> Self {
        self.bw = Some(bw);
        self
    }

    /// Set the power mode
    pub fn mode(mut self, mode: PowerMode) -> Self {
        self.mode = Some(mode);
        self
    }

    /// Build the MagConfig
    pub fn build(self) -> MagConfig {
        MagConfig {
            odr: self.odr.unwrap_or(DataRate::ODR100Hz),
            performance: self.performance.unwrap_or(PerformanceMode::Regular),
            bw: self.bw.unwrap_or(Bandwidth::Normal),
            mode: self.mode.unwrap_or(PowerMode::Normal),
        }
    }
}

impl From<MagConfig> for u16 {
    /// Convert MagConfig to a 16-bit register value
    fn from(config: MagConfig) -> Self {
        (config.odr as u16 & 0x0F)
            | ((config.performance as u16 & 0x03) << 4)
            | ((config.bw as u16 & 0x01) << 7)
            | ((config.mode as u16 & 0x07) << 12)
    }
}
