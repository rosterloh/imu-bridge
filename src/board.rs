use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::peripherals::{
    GPIO0, GPIO11, GPIO12, GPIO13, GPIO21, GPIO38, GPIO45, GPIO46, GPIO47, GPIO48, I2C0, SPI2,
};
use esp_hal::spi::{
    Mode,
    master::{Config as SpiConfig, Spi},
};
use esp_hal::time::Rate;
use static_cell::StaticCell;

pub type SharedI2c = Mutex<CriticalSectionRawMutex, I2c<'static, esp_hal::Async>>;
pub type SharedSpi = Mutex<CriticalSectionRawMutex, Spi<'static, esp_hal::Async>>;

static I2C_BUS: StaticCell<SharedI2c> = StaticCell::new();
static SPI_BUS: StaticCell<SharedSpi> = StaticCell::new();

pub struct BoardContext {
    pub i2c_bus: &'static SharedI2c,
    pub spi_bus: &'static SharedSpi,
    pub spi_cs_accel: Output<'static>,
    pub spi_cs_gyro: Option<Output<'static>>,
    pub led0: Output<'static>,
}

pub fn init(
    i2c0: I2C0<'static>,
    spi2: SPI2<'static>,
    sda_gpio: GPIO11<'static>,
    scl_gpio: GPIO12<'static>,
    cs0_gpio: GPIO21<'static>,
    cs1_gpio: GPIO13<'static>,
    mosi_gpio: GPIO38<'static>,
    miso_gpio: GPIO47<'static>,
    sck_gpio: GPIO48<'static>,
    led0_gpio: GPIO46<'static>,
    led1_gpio: GPIO0<'static>,
    led2_gpio: GPIO45<'static>,
) -> BoardContext {
    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let i2c0 = I2c::new(i2c0, i2c_config)
        .unwrap()
        .with_sda(sda_gpio)
        .with_scl(scl_gpio)
        .into_async();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c0));

    let spi_config = SpiConfig::default()
        .with_frequency(Rate::from_mhz(5))
        .with_mode(Mode::_0);
    let spi2 = Spi::new(spi2, spi_config)
        .unwrap()
        .with_sck(sck_gpio)
        .with_mosi(mosi_gpio)
        .with_miso(miso_gpio)
        .into_async();
    let spi_bus = SPI_BUS.init(Mutex::new(spi2));
    let cs_accel = Output::new(cs0_gpio, Level::High, OutputConfig::default());
    let cs_gyro = Output::new(cs1_gpio, Level::High, OutputConfig::default());

    let _led_red = Output::new(led0_gpio, Level::High, OutputConfig::default());
    let green_led = Output::new(led1_gpio, Level::High, OutputConfig::default());
    let _led_blue = Output::new(led2_gpio, Level::High, OutputConfig::default());

    return BoardContext {
        i2c_bus,
        spi_bus,
        spi_cs_accel: cs_accel,
        spi_cs_gyro: Some(cs_gyro),
        led0: green_led,
    };
}
