#![no_std]
#![no_main]

use core::cell::RefCell;

use bsp::entry;
use defmt::info;
use defmt_rtt as _;
use embedded_hal::digital::{InputPin, OutputPin, PinState};
use embedded_hal_bus::spi::RefCellDevice;
use panic_probe as _;

use rp_pico::{
    self as bsp,
    hal::{fugit::RateExtU32, gpio::FunctionSpi},
};
use vs1003_driver::{pac::AdpcmInput, ClockBoost, ClockMultiplier, Peripherals, Vs1003};

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

struct BytesMut<'a> {
    buffer: &'a mut [u8],
    offset: usize,
}
impl<'a> From<&'a mut [u8]> for BytesMut<'a> {
    fn from(value: &'a mut [u8]) -> Self {
        Self {
            buffer: value,
            offset: 0,
        }
    }
}
impl<'buf> BytesMut<'buf> {
    pub fn put<'a>(&'a mut self, data: impl AsRef<[u8]>)
    where
        'buf: 'a,
    {
        let data = data.as_ref();
        assert!(data.len() <= (self.buffer.len() - self.offset));

        let written = &mut self.buffer[self.offset..][..data.len()];
        written.copy_from_slice(data);

        self.offset += data.len();
    }
}

#[derive(Clone, Copy)]
struct SharedDelay<'a>(&'a RefCell<cortex_m::delay::Delay>);
impl SharedDelay<'_> {
    fn delay_ms(&self, ms: u32) {
        self.0.borrow_mut().delay_ms(ms);
    }

    fn delay_us(&self, us: u32) {
        self.0.borrow_mut().delay_us(us);
    }
}
impl embedded_hal::delay::DelayNs for SharedDelay<'_> {
    fn delay_ns(&mut self, ns: u32) {
        self.0.borrow_mut().delay_us((ns + 999) / 1000);
    }
}

fn make_wav_header(buffer: &mut [u8], data_length: u32) {
    let file_length = data_length + 44;
    let mut buffer = BytesMut::from(buffer);
    buffer.put(b"RIFF");
    buffer.put((file_length - 8).to_le_bytes());
    buffer.put(b"WAVE");
    buffer.put(b"fmt ");
    buffer.put(16u32.to_le_bytes());
    // PCM data
    buffer.put(1u16.to_le_bytes());
    // 2 channels
    buffer.put(2u16.to_le_bytes());
    // Sample rate
    buffer.put(48000u32.to_le_bytes());
    // Byterate - sample rate * channels * bytes per sample
    buffer.put((48000u32 * 2 * 2).to_le_bytes());
    // Byte alignement - channels * bytes per sample
    buffer.put((2u16 * 2).to_le_bytes());
    // Bits per sample
    buffer.put(16u16.to_le_bytes());
    buffer.put(b"data");
    buffer.put((file_length - 44).to_le_bytes());
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let delay = RefCell::new(cortex_m::delay::Delay::new(
        core.SYST,
        clocks.system_clock.freq().to_Hz(),
    ));
    let mut delay = SharedDelay(&delay);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sck = pins.gpio2.into_function::<FunctionSpi>();
    let mosi = pins.gpio3.into_function::<FunctionSpi>();
    let miso = pins.gpio4.into_function::<FunctionSpi>();
    let xcs = pins.gpio5.into_push_pull_output_in_state(PinState::High);
    let mut dreq = pins.gpio6.into_pull_up_input();
    let mut xrst = pins.gpio7.into_push_pull_output_in_state(PinState::High);
    let xdcs = pins.gpio8.into_push_pull_output_in_state(PinState::High);

    let mut led_pin = pins.led.into_push_pull_output();

    info!("Resetting VS1003");
    xrst.set_low().unwrap();
    delay.delay_ms(1);
    xrst.set_high().unwrap();
    delay.delay_us(100);
    info!("dreq state: {:?}", dreq.is_high());
    delay.delay_ms(2);

    let spi0 = bsp::hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        1_000_000u32.Hz(),
        embedded_hal::spi::MODE_0,
    );
    let spi0 = RefCell::new(spi0);
    let sci = RefCellDevice::new(&spi0, xcs, delay).unwrap();
    let sdi = RefCellDevice::new(&spi0, xdcs, delay).unwrap();
    let dev = Vs1003::new(Peripherals {
        sci,
        sdi,
        dreq,
        xrst,
    });

    let mut dev = dev
        .reset_initialize(
            &mut delay,
            12_228.kHz(),
            ClockMultiplier::Times4_0,
            ClockBoost::Plus0_0,
        )
        .unwrap();

    dev.sci()
        .mode()
        .modify(|r| {
            r.set_reset(true);
            r.set_adpcm(true);
            r.set_adpcm_input(AdpcmInput::Microphone);
        })
        .unwrap();

    while let Ok(true) = dev.is_busy() {}

    spi0.borrow_mut()
        .set_baudrate(clocks.peripheral_clock.freq(), 10.MHz());

    const SECONDS: u32 = 10;
    const TOTAL_SAMPLES: u32 = SECONDS * 48000;
    const BUFFER_SIZE: u32 = 128;
    let mut buffer = [0u8; BUFFER_SIZE as usize];
    make_wav_header(&mut buffer, 4 * TOTAL_SAMPLES);

    let mut current_sample = 0u32;
    let mut next_report = 0;

    while current_sample < TOTAL_SAMPLES {
        // Generate samples
        let start = if current_sample == 0 { 44 } else { 0 };
        let mut buffer_mut = BytesMut::from(&mut buffer[start..]);
        for i in (start as u32 / 4)..(BUFFER_SIZE / 4) {
            let sample_value = (((current_sample + i) / 64) & 0xFFFF) as u16;
            buffer_mut.put(sample_value.to_le_bytes());
            buffer_mut.put((sample_value.wrapping_add(512)).to_le_bytes());
        }

        let mut written_so_far = 0;
        while written_so_far < buffer.len() {
            while dev.is_busy().unwrap() {}
            written_so_far += dev.send_data(&buffer[written_so_far..]).unwrap();
        }

        if current_sample > next_report {
            next_report += 48000 / 2;
            let vs_status = dev.sci().status().read();
            while let Ok(true) = dev.is_busy() {}
            let hd0 = dev.sci().hdat_0().read();
            while let Ok(true) = dev.is_busy() {}
            let decode_time = dev.sci().decode_time().read();
            info!(
                "play, status: {:?}, hdat0: {:?}, decode_time: {:?}",
                vs_status, hd0, decode_time
            );
        }

        current_sample += BUFFER_SIZE / 4;
    }

    loop {
        let vs_status = dev.sci().status().read();
        while let Ok(true) = dev.is_busy() {}
        let hd0 = dev.sci().hdat_0().read();
        while let Ok(true) = dev.is_busy() {}
        let decode_time = dev.sci().decode_time().read();
        info!(
            "on!, status: {:?}, hdat0: {:?}, decode_time: {:?}",
            vs_status, hd0, decode_time
        );

        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}
