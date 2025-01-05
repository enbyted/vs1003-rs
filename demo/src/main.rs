#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    num::NonZeroU16,
    ops::{Deref, DerefMut},
    ptr::addr_of_mut,
};

use bsp::entry;
use defmt::{error, info, warn};
use defmt_rtt as _;
use embedded_hal::digital::{InputPin, OutputPin, PinState};
use embedded_hal_bus::spi::RefCellDevice;
use fugit::HertzU32;
use once_cell::sync::OnceCell;
use panic_probe as _;

use rp_pico::{
    self as bsp,
    hal::{fugit::RateExtU32, gpio::FunctionSpi},
};
use vs1003_driver::{
    ClockBoost, ClockMultiplier, ModeChangeError, Peripherals, RecordingFilter, RecordingGain,
    RecordingInput, Vs1003,
};

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
impl Deref for BytesMut<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.buffer[self.offset..]
    }
}
impl DerefMut for BytesMut<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer[self.offset..]
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

fn make_wav_adpcm_header(buffer: &mut [u8], sample_frequency: HertzU32, data_length: u32) {
    let file_length = data_length + 60;
    let mut buffer = BytesMut::from(buffer);
    buffer.put(b"RIFF");
    buffer.put((file_length - 8).to_le_bytes());
    buffer.put(b"WAVE");
    buffer.put(b"fmt ");
    buffer.put(20u32.to_le_bytes());
    // IMA ADPCM data
    buffer.put(0x11u16.to_le_bytes());
    // 1 channel
    buffer.put(1u16.to_le_bytes());
    // Sample rate
    buffer.put(sample_frequency.to_Hz().to_le_bytes());
    // Byterat
    buffer.put((sample_frequency.to_Hz() * 256 / 505).to_le_bytes());
    // Block align
    buffer.put(0x0100u16.to_le_bytes());
    // Bits per sample
    buffer.put(4u16.to_le_bytes());
    // Extra data
    buffer.put(2u16.to_le_bytes());
    // Samples per block
    buffer.put(505u16.to_le_bytes());
    buffer.put(b"fact");
    buffer.put(4u32.to_le_bytes());
    // Number of samples
    buffer.put((data_length / 256 * 505).to_le_bytes());
    buffer.put(b"data");
    buffer.put((file_length - 60).to_le_bytes());
    assert_eq!(buffer.offset, 60);
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

    static TIMER: OnceCell<rp_pico::hal::Timer> = OnceCell::new();
    TIMER
        .set(rp_pico::hal::Timer::new(
            pac.TIMER,
            &mut pac.RESETS,
            &clocks,
        ))
        .ok()
        .unwrap();

    defmt::timestamp!("{=u32:us}", {
        // TODO: Why does this hang? Problably something like taking a critical section inside a critical secion
        // TIMER.get().unwrap().get_counter().ticks()
        // For now let's just read the timer register
        unsafe { &*rp_pico::pac::TIMER::PTR }
            .timerawl()
            .read()
            .bits()
    });

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

    spi0.borrow_mut()
        .set_baudrate(clocks.peripheral_clock.freq(), 1.MHz());

    // Try to play something
    // const SECONDS: u32 = 2;
    // const TOTAL_SAMPLES: u32 = SECONDS * 48000;
    // const BUFFER_SIZE: u32 = 128;
    // let mut buffer = [0u8; BUFFER_SIZE as usize];
    // make_wav_header(&mut buffer, 4 * TOTAL_SAMPLES);

    // let mut current_sample = 0u32;
    // let mut next_report = 0;

    // while current_sample < TOTAL_SAMPLES {
    //     // Generate samples
    //     let start = if current_sample == 0 { 44 } else { 0 };
    //     let mut buffer_mut = BytesMut::from(&mut buffer[start..]);
    //     for i in (start as u32 / 4)..(BUFFER_SIZE / 4) {
    //         let sample_value = (((current_sample + i) / 64) & 0xFFFF) as u16;
    //         buffer_mut.put(sample_value.to_le_bytes());
    //         buffer_mut.put((sample_value.wrapping_add(512)).to_le_bytes());
    //     }

    //     let mut written_so_far = 0;
    //     while written_so_far < buffer.len() {
    //         while dev.is_busy().unwrap() {}
    //         written_so_far += dev.send_data(&buffer[written_so_far..]).unwrap();
    //     }

    //     if current_sample > next_report {
    //         next_report += 48000 / 2;
    //         let vs_status = dev.sci().status().read();
    //         while let Ok(true) = dev.is_busy() {}
    //         let hd0 = dev.sci().hdat_0().read();
    //         while let Ok(true) = dev.is_busy() {}
    //         let decode_time = dev.sci().decode_time().read();
    //         info!(
    //             "play, status: {:?}, hdat0: {:?}, decode_time: {:?}",
    //             vs_status, hd0, decode_time
    //         );
    //     }

    //     current_sample += BUFFER_SIZE / 4;
    // }

    // Try to play MP3

    // let test_mp3 = include_bytes!("sample-file-4.mp3");
    // let mut remaining = test_mp3.as_slice();

    // let mut last_update = TIMER.get().unwrap().get_counter();
    // while !remaining.is_empty() {
    //     while let Ok(true) = dev.is_busy() {}
    //     let written = dev.send_data(remaining).unwrap();

    //     let now = TIMER.get().unwrap().get_counter();
    //     if (now - last_update).to_millis() >= 500 {
    //         let vs_status = dev.sci().status().read();
    //         while let Ok(true) = dev.is_busy() {}
    //         let hd0 = dev.sci().hdat_0().read();
    //         while let Ok(true) = dev.is_busy() {}
    //         let decode_time = dev.sci().decode_time().read();
    //         info!(
    //             "play, status: {:?}, hdat0: {:?}, decode_time: {:?}",
    //             vs_status, hd0, decode_time
    //         );
    //         last_update = now;
    //     }
    //     remaining = &remaining[written..];
    // }

    // const SAMPLES_PER_BLOCK: usize = (256 - 4) * 2;
    // let mut read_buffer = [0u16; 128];
    // let mut read_blocks = 0;
    // let mut next_500ms = 0;

    // dev.sci()
    //     .read_all_registers(|addr, name, value| {
    //         info!("Reg({:X}h) {}: {:?}", addr, name, value);
    //     })
    //     .unwrap();
    // info!(
    //     "Mode: {:?}, ai0: {:?}, ai1: {:?}",
    //     dev.sci().mode().read(),
    //     dev.sci().ai_ctrl(0).read(),
    //     dev.sci().ai_ctrl(1).read()
    // );

    const SAMPLE_FREQ: HertzU32 = HertzU32::from_raw(48_000);
    const RECORD_BUFFER_SIZE: usize = 100 * 1024;
    static mut RECORD_BUFFER: [u8; RECORD_BUFFER_SIZE] = [0; RECORD_BUFFER_SIZE];

    let record_buffer = unsafe { &mut *addr_of_mut!(RECORD_BUFFER) };

    // A simple example that first records some data and then plays it back (with current settings ~4.5s)
    loop {
        // First - record data
        info!("Recording");
        led_pin.set_high().unwrap();
        let mut buffer = BytesMut::from(&mut record_buffer[60..]);

        while let Ok(true) = dev.is_busy() {}
        let mut record_dev = dev
            .begin_adpcm_recording(
                &mut delay,
                SAMPLE_FREQ,
                RecordingInput::Microphone,
                RecordingGain::Manual(NonZeroU16::new(8 * 1024).unwrap()),
                RecordingFilter::NoFilter,
            )
            .unwrap();

        while buffer.len() >= 256 {
            while let Ok(true) = record_dev.is_busy() {}

            let available_words = record_dev.sci().hdat_1().read().unwrap().value();
            if available_words >= 128 {
                for _ in 0..128 {
                    while let Ok(true) = record_dev.is_busy() {}
                    let sample = record_dev.sci().hdat_0().read().unwrap().value();
                    buffer.put(sample.to_be_bytes());
                }
            }
            delay.delay_us(5);
        }

        let used_buffer = RECORD_BUFFER_SIZE - buffer.len();
        let data_buffer: &mut [u8] = &mut record_buffer[..(used_buffer + 60)];

        make_wav_adpcm_header(data_buffer, SAMPLE_FREQ, used_buffer as u32);
        info!("Recorded {} bytes", used_buffer);

        led_pin.set_low().unwrap();

        let mut error_dev = record_dev.into_errored_state();
        dev = loop {
            let result = error_dev.reset_initialize(
                &mut delay,
                12_228.kHz(),
                ClockMultiplier::Times4_0,
                ClockBoost::Plus0_0,
            );
            match result {
                Ok(player_dev) => break player_dev,
                Err(ModeChangeError {
                    error: vs1003_driver::Error::Busy,
                    device,
                }) => {
                    warn!("Device is busy!");
                    delay.delay_ms(10);
                    error_dev = device;
                }
                Err(other_error) => {
                    error!("Failed to change mode: {}", other_error.error);
                    panic!("Failed to change mode: {:?}", other_error);
                }
            }
        };

        info!("Playing");
        let mut remaining = &data_buffer[..];
        let mut total_written = 0;
        while !remaining.is_empty() {
            while let Ok(true) = dev.is_busy() {}
            let written = dev
                .send_data(remaining)
                .or_else(|e| match e {
                    vs1003_driver::Error::Busy => Ok(0),
                    other => Err(other),
                })
                .unwrap();
            total_written += written;
            remaining = &remaining[written..];
        }

        info!("Total played {} bytes", total_written);
    }
}
