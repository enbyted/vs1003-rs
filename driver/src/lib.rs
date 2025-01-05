//! TODO: Write some docs
//!

#![no_std]
#![warn(missing_debug_implementations)]
#![warn(missing_docs)]

pub use vs1003_pac as pac;

use embedded_hal::{delay::DelayNs, digital::OutputPin as _, spi::SpiDevice};
use fugit::ExtU32;

/// Contains all peripherals required to interface with VS1003
#[derive(Debug)]
pub struct Peripherals<TSci, TSdi, TDreq, TRst> {
    /// The SCI SPI slave.
    /// Typically that will be the same SPI bus as SDI, with different CS (xCS).
    pub sci: TSci,
    /// The SDI SPI Slave.
    /// Typically that will be the same SPI bus as SCI, with different CS (xDCS).
    pub sdi: TSdi,
    /// The DREQ input pin.
    pub dreq: TDreq,
    /// The xRST output pin.
    pub xrst: TRst,
}

impl<TSci, TSdi, TDreq, TRst> Vs1003Peripherals for Peripherals<TSci, TSdi, TDreq, TRst>
where
    TSci: embedded_hal::spi::SpiDevice,
    TSdi: embedded_hal::spi::SpiDevice,
    TDreq: embedded_hal::digital::InputPin,
    TRst: embedded_hal::digital::OutputPin,
{
    type TSci = TSci;
    type TSdi = TSdi;
    type TDreq = TDreq;
    type TRst = TRst;
    type Error = Error<TSci::Error, TSdi::Error, TDreq::Error, TRst::Error>;

    fn take(self) -> Self {
        self
    }
}

/// Helper trait to represent device peripherals.
///
/// This is used in the implmenetation to reduce the number of generic arguments everywhere.
pub trait Vs1003Peripherals {
    /// The type of the command interface SPI slave
    type TSci: embedded_hal::spi::SpiDevice;
    /// The type of the data interface SPI slave
    type TSdi: embedded_hal::spi::SpiDevice;
    /// The type of the DREQ input pin
    type TDreq: embedded_hal::digital::InputPin;
    /// Tyhe type of the xRST output pin
    type TRst: embedded_hal::digital::OutputPin;

    /// The error type that will be returned by the driver
    type Error: core::fmt::Debug
        + core::error::Error
        + From<
            Error<
                <Self::TSci as embedded_hal::spi::ErrorType>::Error,
                <Self::TSdi as embedded_hal::spi::ErrorType>::Error,
                <Self::TDreq as embedded_hal::digital::ErrorType>::Error,
                <Self::TRst as embedded_hal::digital::ErrorType>::Error,
            >,
        >;

    /// Return the peripheral instances
    fn take(self) -> Peripherals<Self::TSci, Self::TSdi, Self::TDreq, Self::TRst>;
}

#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
/// The error enum returned by most of [Vs1003] APIs
pub enum Error<ESci, ESdi, EDreq, ERst> {
    /// One of the methods on SCI interface failed
    #[error("Failed to communicate over SCI")]
    Sci(ESci),

    /// One of the methods on SDI interface failed
    #[error("Failed to communicate over SDI")]
    Sdi(ESdi),

    /// Reading of DREQ pin state failed
    #[error("Failed to read DREQ signal")]
    DreqRead(EDreq),

    /// Setting xRST state failed
    #[error("Failed to write xRST signal")]
    RstWrite(ERst),

    /// Attempted to perform a write while DREQ was low
    #[error("Device is busy (DREQ is low)")]
    Busy,

    /// Operation has timed out, likely the DREQ signal didn't rise when expected
    #[error("Timeout of {}us exceeded", .0.ticks())]
    Timeout(fugit::MicrosDurationU32),
}
impl<ESci, ESdi, EDreq, ERst> From<pac::Vs1003InterfaceError<ESci, EDreq>>
    for Error<ESci, ESdi, EDreq, ERst>
{
    fn from(value: pac::Vs1003InterfaceError<ESci, EDreq>) -> Self {
        match value {
            pac::Vs1003InterfaceError::Spi(err) => Self::Sci(err),
            pac::Vs1003InterfaceError::Dreq(err) => Self::DreqRead(err),
            pac::Vs1003InterfaceError::Busy => Self::Busy,
        }
    }
}

#[derive(thiserror::Error)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
/// The error returned by [Vs1003] APIs that change the typesate
pub struct ModeChangeError<T: Vs1003Peripherals> {
    /// The actual error
    pub error: T::Error,
    /// The device, so that it can be reset
    pub device: Vs1003<Errored, T>,
}
impl<T: Vs1003Peripherals> core::fmt::Debug for ModeChangeError<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // Too bad we can't provide different debug output depending on whether Vs1003<Errored, T> is Debug or not :(
        f.debug_tuple("ModeChangeError").field(&self.error).finish()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
/// Specifies the clock multiplier used by the VS1003
pub enum ClockMultiplier {
    /// Use base input clock, no multiplication
    Times1_0 = 0,
    /// 1.5x
    Times1_5 = 1,
    /// 2.0x
    Times2_0 = 2,
    /// 2.5x
    Times2_5 = 3,
    /// 3.0x
    Times3_0 = 4,
    /// 3.5x
    Times3_5 = 5,
    /// 4.0x
    Times4_0 = 6,
    /// 4.5x
    ///
    /// Note: with the default clock of 12.228MHz this will overclock the device
    Times4_5 = 7,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
/// Specifies the maximum clock boost (i.e. multiplier added to [ClockMultiplier])
/// that can be used for WMA decoding.
pub enum ClockBoost {
    /// Do not boost the clock
    Plus0_0 = 0,
    /// Clock may be boosted up to [ClockMultiplier] + 0.5x
    Plus0_5 = 1,
    /// Clock may be boosted up to [ClockMultiplier] + 1.0x
    Plus1_0 = 2,
    /// Clock may be boosted up to [ClockMultiplier] + 1.5x
    Plus1_5 = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
/// Specifies the input used for recording
pub enum RecordingInput {
    /// Use the LINEIN input pin
    LineIn,
    /// Use the MICP/MICN input pair
    Microphone,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
/// Specifies the digital gain to use for recording
pub enum RecordingGain {
    /// Use automatic gain control
    Auto,
    /// Manual gain in range 64x - ~0.0001x where 1024 = 1x
    Manual(core::num::NonZeroU16),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
/// Specifies whether to use a high-pass filter for recording or not
pub enum RecordingFilter {
    /// No filtering of lower-band
    NoFilter,
    /// ~300Hz high pass filter.
    ///
    /// Note that the datasheet says that this is only usable with 8kHz sample rate.
    HighPass,
}

/// Typestate type for freshly constructed VS1003
#[derive(Debug)]
#[non_exhaustive]
pub struct NotInitialized {}

/// Typestate type for a VS1003 that failed state change
#[derive(Debug)]
#[non_exhaustive]
pub struct Errored {}

/// Typestate type for freshly reset VS1003
#[derive(Debug)]
#[non_exhaustive]
pub struct Initialized {
    /// The internal clock speed
    internal_clock: fugit::KilohertzU32,
}

/// Typestate type for VS1003 in standard recording mode
#[derive(Debug)]
#[non_exhaustive]
pub struct AdpcmRecording {}

#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
/// High level interface for the VS1003 audio codec
pub struct Vs1003<State, T: Vs1003Peripherals> {
    sci: pac::Vs1003<pac::Vs1003Interface<T::TSci, T::TDreq>>,
    sdi: T::TSdi,
    xrst: T::TRst,
    state: State,
}

impl<T: Vs1003Peripherals> Vs1003<NotInitialized, T> {
    /// Create the interface
    pub fn new(peripherals: T) -> Self {
        let Peripherals {
            sci,
            sdi,
            dreq,
            xrst,
        } = peripherals.take();

        Self {
            sci: pac::Vs1003::new(pac::Vs1003Interface::new(sci, dreq)),
            sdi,
            xrst,
            state: NotInitialized {},
        }
    }
}

impl<State, T: Vs1003Peripherals> Vs1003<State, T> {
    /// Reset the VS1003 and initialize it's clock settings.
    /// Per datasheet the device supports clock rates between 12 and 13 MHz,
    /// however the configuration registers allow range 8.001 MHz - 270.14 MHz.
    ///
    /// The datasheet also notes that the a minimum clock of 12.228 MHz is needed
    /// to support the maximum sampling frequency od 48kHz.
    ///
    /// Note: before executing this method the SCI clock speed must be set lower than `clki / 4`.
    /// After executing this method the clock speed may be increased `base_multiplier` times.
    ///
    /// # Blocking
    ///
    /// This method may block for up to 50ms, assuming that `delay.delay_us(10)` blocks for 10us and a reasonable SPI clock speed.
    ///
    /// # Panics
    ///
    /// This method will panic in the provided clock frequency is outside
    /// of legal range for VS1003.
    pub fn reset_initialize(
        mut self,
        delay: &mut impl DelayNs,
        xtali: fugit::KilohertzU32,
        base_multiplier: ClockMultiplier,
        wma_boost: ClockBoost,
    ) -> Result<Vs1003<Initialized, T>, ModeChangeError<T>> {
        let xtali_4k: u32 = xtali.to_kHz() / 4;
        assert!(xtali_4k > 2_000, "Provided frequency is too low!");
        let clock_input = xtali_4k - 2_000;
        assert!(
            clock_input < u16::MAX as u32,
            "Provided frequency is too high!"
        );
        let clock_input = clock_input as u16;
        let internal_clock = xtali
            * match base_multiplier {
                // 2x the actual clock, to avoid using floating point
                ClockMultiplier::Times1_0 => 2,
                ClockMultiplier::Times1_5 => 3,
                ClockMultiplier::Times2_0 => 4,
                ClockMultiplier::Times2_5 => 5,
                ClockMultiplier::Times3_0 => 6,
                ClockMultiplier::Times3_5 => 7,
                ClockMultiplier::Times4_0 => 8,
                ClockMultiplier::Times4_5 => 9,
            }
            / 2;

        let mut do_reset = move |dev: &mut Self| -> Result<(), T::Error> {
            dev.xrst.set_low().map_err(Error::RstWrite)?;
            delay.delay_us(100);
            dev.xrst.set_high().map_err(Error::RstWrite)?;
            delay.delay_ms(5);
            // According to datasheet the maximum reset time is 50000 clock cycles
            // At 12MHz this is ~4.2ms, at 8MHz it would be 6.25ms
            // Thus 10ms seems like a good timeout to use
            dev.wait_for_dreq(delay, 10.millis())?;

            dev.sci()
                .clockf()
                .modify(move |r| {
                    r.set_multiplier(base_multiplier as u8);
                    r.set_allowed_addition(wma_boost as u8);
                    r.set_input_frequency(clock_input);
                })
                .map_err(Error::from)?;

            // Datasheet specifies maximum execution time of 11000 clock cycles
            // At 12MHz this is ~1ms, at 12 MHz ~1.4ms.
            // Thus 5ms timeout seems like a good compromise
            dev.wait_for_dreq(delay, 5.millis())?;
            Ok(())
        };

        match do_reset(&mut self) {
            Err(error) => Err(ModeChangeError {
                error,
                device: Vs1003 {
                    sci: self.sci,
                    sdi: self.sdi,
                    xrst: self.xrst,
                    state: Errored {},
                },
            }),
            Ok(_) => Ok(Vs1003 {
                sci: self.sci,
                sdi: self.sdi,
                xrst: self.xrst,
                state: Initialized { internal_clock },
            }),
        }
    }

    /// Proxy for [pac::Vs1003::is_busy].
    /// Returns true when the device cannot accept commands.
    pub fn is_busy(&mut self) -> Result<bool, T::Error> {
        Ok(self.sci.is_busy().map_err(Error::DreqRead)?)
    }

    /// Get the SCI interface to allow custom configuration of the device
    pub fn sci(&mut self) -> &mut pac::Vs1003<pac::Vs1003Interface<T::TSci, T::TDreq>> {
        &mut self.sci
    }

    fn wait_for_dreq(
        &mut self,
        delay: &mut impl embedded_hal::delay::DelayNs,
        max_delay: fugit::MicrosDurationU32,
    ) -> Result<(), T::Error> {
        const DELAY_STEP: u32 = 10;

        let mut remaining_delay_us = max_delay.ticks();

        while remaining_delay_us > 0 {
            delay.delay_us(remaining_delay_us.min(DELAY_STEP));
            if !self.is_busy()? {
                return Ok(());
            }
            remaining_delay_us = remaining_delay_us.saturating_sub(DELAY_STEP);
        }

        Err(Error::Timeout(max_delay))?
    }

    fn change_state<NewState>(self, state: NewState) -> Vs1003<NewState, T> {
        Vs1003 {
            sci: self.sci,
            sdi: self.sdi,
            xrst: self.xrst,
            state,
        }
    }
}

impl<T: Vs1003Peripherals> Vs1003<Initialized, T> {
    /// Enter the ADPCM recording mode of the device.
    ///
    /// In this mode the device will sample the input at provided rate and encode it as ADPCM
    ///
    /// The actual sample rate may differ from requested due to divider resolution.
    ///
    /// # Blocking
    ///
    /// This method may block for up to 25ms, assuming that `delay.delay_us(10)` blocks for 10us and a reasonable SPI clock speed.
    ///
    /// # Panics
    ///
    /// This call will panic if the requested rate is higher than can be achieved with the configured clock.
    /// The maximum achievable rate is CLKI / 1024.
    ///
    /// This
    pub fn begin_adpcm_recording(
        mut self,
        delay: &mut impl DelayNs,
        sample_rate: fugit::HertzU32,
        input: RecordingInput,
        gain: RecordingGain,
        filter: RecordingFilter,
    ) -> Result<Vs1003<AdpcmRecording, T>, ModeChangeError<T>> {
        let divider = self
            .state
            .internal_clock
            .to_Hz()
            .div_ceil(256 * sample_rate.to_Hz());
        defmt::info!(
            "Sample rate: {:?}, clock: {:?}, divider: {}",
            sample_rate,
            self.state.internal_clock,
            divider
        );
        assert!(
            divider >= 4,
            "Requested sample rate is too high for provided clock"
        );
        let mut do_change = move |dev: &mut Self| -> Result<(), T::Error> {
            // AICTRL0 is the divider
            dev.sci
                .ai_ctrl(0)
                .write(|r| r.set_value(divider as u16))
                .map_err(Error::from)?;

            dev.wait_for_dreq(delay, 500.micros())?;

            // AICTRL1 is the gain
            dev.sci
                .ai_ctrl(1)
                .write(|r| {
                    r.set_value(match gain {
                        RecordingGain::Auto => 0,
                        RecordingGain::Manual(non_zero) => non_zero.get(),
                    })
                })
                .map_err(Error::from)?;

            dev.wait_for_dreq(delay, 500.micros())?;

            // For VS1033 AICTRL2 is the maximum gain in AGC mode. We won't write to it here, so that
            // the user may set it before calling us. The default of 0 is maximum of 64x, which is what VS1003 implements.

            dev.sci
                .mode()
                .modify(|r| {
                    r.set_reset(true);
                    r.set_adpcm(true);
                    r.set_adpcm_hp(matches!(filter, RecordingFilter::HighPass));
                    r.set_adpcm_input(match input {
                        RecordingInput::LineIn => pac::AdpcmInput::LineIn,
                        RecordingInput::Microphone => pac::AdpcmInput::Microphone,
                    });
                })
                .map_err(Error::from)?;

            dev.wait_for_dreq(delay, 10.millis())?;
            Ok(())
        };

        match do_change(&mut self) {
            Err(error) => Err(ModeChangeError {
                error,
                device: self.change_state(Errored {}),
            }),
            Ok(_) => Ok(self.change_state(AdpcmRecording {})),
        }
    }

    /// Write audio file data to the device. This will send as much data as the device will accept
    /// and return the number of bytes written.
    ///
    /// If DREQ is low on entry then returns [Error::Busy]
    pub fn send_data(&mut self, buffer: &[u8]) -> Result<usize, T::Error> {
        if self.is_busy()? {
            return Err(Error::Busy)?;
        }

        // Datasheet specifies that low DREQ means that the device can receive at least 32 bytes of data
        const CHUNK_SIZE: usize = 32;

        let mut remaining = buffer;

        while !remaining.is_empty() {
            let (taken, leftover) = remaining.split_at(CHUNK_SIZE.min(remaining.len()));
            remaining = leftover;

            self.sdi.write(taken).map_err(Error::Sdi)?;

            // If DREQ fell during the transfer we shal not transfer any more data
            if self.is_busy()? {
                break;
            }
        }

        Ok(buffer.len() - remaining.len())
    }

    /// Changes the typestate to errored.
    /// Can be useful for simplification of retry loops.
    pub fn into_errored_state(self) -> Vs1003<Errored, T> {
        self.change_state(Errored {})
    }
}

impl<T: Vs1003Peripherals> Vs1003<AdpcmRecording, T> {
    /// Changes the typestate to errored.
    /// Can be useful for simplification of retry loops.
    pub fn into_errored_state(self) -> Vs1003<Errored, T> {
        self.change_state(Errored {})
    }
}
