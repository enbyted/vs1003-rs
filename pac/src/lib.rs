#![no_std]
#![warn(missing_debug_implementations)]
#![warn(missing_docs)]

//! The low-level peripheral access definitions for VS1003 codec.
//! Some of those definition may also work for other chips in the family, however
//! there are some incompatible changes between them.
//!
//! Each method checks the DREQ pin and if it is low it will return [`Vs1003InterfaceError::Busy`] if so.
//! Please note though, that the device may not lower this pin immediately after a successful transfer.
//! This time doesn't seem well-specified, however on the forums they seem to recommend a 1us delay after
//! completion of a command and before checking the DREQ pin.

#[derive(Debug, Clone)]
/// The necessary interfaces to communicate with a VS1003 chip.
pub struct Vs1003Interface<TCsi, TDreq> {
    csi: TCsi,
    dreq: TDreq,
}

#[derive(thiserror::Error, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
/// The error type that can be returned by various interface APIs
pub enum Vs1003InterfaceError<ESpi, EDreq> {
    /// The SPI peripheral has failed to compelte a transfer
    #[error("SPI error")]
    Spi(#[source] ESpi),
    /// The Digital input failed to provide a value
    #[error("Digital input error")]
    Dreq(#[source] EDreq),
    /// The DREQ signal is low and the command cannot be sent
    #[error("VS1003 is busy (DREQ is low)")]
    Busy,
}

impl<TCsi, TDreq> Vs1003Interface<TCsi, TDreq> {
    /// Create a new interface.
    ///
    /// While you can pass anything here tou should pass appropriate HAL
    /// stucts that implement necessary traits from [`embedded_hal`] and/or [`embedded_hal_async`].
    pub const fn new(interface: TCsi, dreq: TDreq) -> Self {
        Vs1003Interface {
            csi: interface,
            dreq,
        }
    }
}

impl<TCsi, TDreq> Vs1003Interface<TCsi, TDreq>
where
    TDreq: embedded_hal::digital::InputPin,
{
    /// Check if the device can accept commands or data
    pub fn is_busy(&mut self) -> Result<bool, TDreq::Error> {
        self.dreq.is_low()
    }
}

impl<TCsi, TDreq> Vs1003Interface<TCsi, TDreq>
where
    TDreq: embedded_hal_async::digital::Wait,
{
    /// Wait until the device is ready to accept commands or data.
    /// If the device is ready returns immediately
    pub async fn wait_until_ready(&mut self) -> Result<(), TDreq::Error> {
        self.dreq.wait_for_high().await
    }
}

impl<TCsi, TDreq> device_driver::RegisterInterface for Vs1003Interface<TCsi, TDreq>
where
    TCsi: embedded_hal::spi::SpiDevice<u8>,
    TDreq: embedded_hal::digital::InputPin,
{
    type Error = Vs1003InterfaceError<TCsi::Error, TDreq::Error>;
    type AddressType = u8;

    fn write_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        assert_eq!(size_bits, 16);
        assert_eq!(data.len(), 2);

        if !self.dreq.is_high().map_err(Vs1003InterfaceError::Dreq)? {
            return Err(Vs1003InterfaceError::Busy);
        }

        let setup = [2u8, address, data[0], data[1]];

        self.csi.write(&setup).map_err(Vs1003InterfaceError::Spi)
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation as Op;

        assert_eq!(size_bits, 16);
        assert_eq!(data.len(), 2);

        if !self.dreq.is_high().map_err(Vs1003InterfaceError::Dreq)? {
            return Err(Vs1003InterfaceError::Busy);
        }

        let setup = [3u8, address];
        self.csi
            .transaction(&mut [Op::Write(&setup), Op::Read(data)])
            .map_err(Vs1003InterfaceError::Spi)
    }
}

impl<TCsi, TDreq> device_driver::AsyncRegisterInterface for Vs1003Interface<TCsi, TDreq>
where
    TCsi: embedded_hal_async::spi::SpiDevice<u8>,
    TDreq: embedded_hal::digital::InputPin,
{
    type Error = Vs1003InterfaceError<TCsi::Error, TDreq::Error>;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        assert_eq!(size_bits, 16);
        assert_eq!(data.len(), 2);

        if !self.dreq.is_high().map_err(Vs1003InterfaceError::Dreq)? {
            return Err(Vs1003InterfaceError::Busy);
        }

        let setup = [2u8, address, data[0], data[1]];

        self.csi
            .write(&setup)
            .await
            .map_err(Vs1003InterfaceError::Spi)
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation as Op;

        assert_eq!(size_bits, 16);
        assert_eq!(data.len(), 2);

        if !self.dreq.is_high().map_err(Vs1003InterfaceError::Dreq)? {
            return Err(Vs1003InterfaceError::Busy);
        }

        let setup = [3u8, address];
        self.csi
            .transaction(&mut [Op::Write(&setup), Op::Read(data)])
            .await
            .map_err(Vs1003InterfaceError::Spi)
    }
}

impl<TCsi, TDreq> Vs1003<Vs1003Interface<TCsi, TDreq>>
where
    TDreq: embedded_hal::digital::InputPin,
{
    /// Check if the device is ready to accept commands or data
    ///
    /// Other functions may fail with [`Vs1003InterfaceError::Busy`] if this returns `Ok(false)`
    pub fn is_busy(&mut self) -> Result<bool, TDreq::Error> {
        self.interface.is_busy()
    }
}

impl<TCsi, TDreq> Vs1003<Vs1003Interface<TCsi, TDreq>>
where
    TDreq: embedded_hal::digital::InputPin + embedded_hal_async::digital::Wait,
{
    /// Asynchronously waits until the device is ready to accept commands and data
    pub async fn wait_until_ready(&mut self) -> Result<(), TDreq::Error> {
        self.interface.dreq.wait_for_high().await
    }
}

device_driver::create_device!(
    device_name: Vs1003,
    dsl: {
        config {
            type RegisterAddressType = u8;
            type DefaultByteOrder = BE;
            type DefmtFeature = "defmt-03";
        }
        /// Access any register to read or write a custom value.
        ///
        /// This is especially useful when uploading plugins as they are specified in
        /// register address + value to write format.
        register Raw {
            const ADDRESS = 0x1;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;
            const REPEAT = {
                count: 16,
                stride: 1,
            };

            /// The value
            value: uint = 0..16,
        },
        /// The SCI_MODE register - controls various aspects of operation of the VS1003
        register Mode {
            const ADDRESS = 0x0;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// If true invert left channel for differential output
            differential: bool = 0,

            /// Reserved - always set to zero.
            reserved: bool = 1,

            /// Set to true to initiate a software reset.
            reset: bool = 2,

            /// Jump out of WAV decoding
            out_of_wav: bool = 3,

            power_down: bool = 4,

            /// Allow SDI tests
            allow_tests: bool = 5,

            /// Enable stream mode
            stream_mode: bool = 6,

            /// Reserved - always set to zero.
            reserved_2: bool = 7,

            /// Active edge for SDI interface
            dclk_active_edge: uint as enum DclkEdge {
                Rising = 0,
                Falling = 1,
            } = 8..=8,

            /// The bit order for the SDI interface
            sdi_bit_order: uint as enum SdiBitOrder {
                MsbFirst = 0,
                LsbFirst = 1,
            } = 9..=9,

            /// Whether to share xCS with xDCS (i.e. SCI & SDI interfaces).
            ///
            /// If true the xDCS signal is generated by inverting xCS.
            sdi_share: bool = 10,

            /// Whether to use VS10xx native mode.
            sdi_new: bool = 11,

            /// Set to true along with reset to start ADPCM recording session.
            adpcm: bool = 12,

            /// Enable or disable high-pass filter for recording.
            adpcm_hp: bool = 13,

            /// The recording input
            adpcm_input: uint as enum AdpcmInput {
                Microphone = 0,
                LineIn = 1,
            } = 14..=14,
        },
        /// The SCI_STATUS register
        register Status {
            const ADDRESS = 0x1;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// The connected device
            version: uint as enum Version {
                Vs1001 = 0,
                Vs1011 = 1,
                Vs1002 = 2,
                Vs1003 = 3,
                Vs1053 = 5,
                Vs1063 = 6,
                Unknown = catch_all
            } = 4..=7,

            /// Controls the analog driver powerdown.
            /// Normally controlled by the system firmware, however
            /// can be set to 1 a few milliseconds before reset to reduce transients.
            analog_driver_powerdown: bool = 3,

            /// Controls internal analog powerdown.
            /// This is meant for system firmware use only.
            analog_internal_powerdown: bool = 2,

            /// Analog volumen control.
            /// This is meant for system firmware use only.
            ///
            /// 0 = -0 dB, 1 = -6 dB, 3 = -12 dB
            analog_volume: uint = 0..=1,
        },

        /// Bass enhancer settings
        register Bass {
            const ADDRESS = 0x2;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Treble control in 1.5 dB steps.
            /// Set to 0 to disable.
            treble_amplitude: int = 12..=15,

            /// Lower limit frequency in 1kHz steps.
            /// ex. When set to 10 the DSP will enhance frequencies >=10kHz
            treble_bottom_frequency: uint = 8..=11,

            /// Bass enhancement in 1 dB steps.
            /// Set to 0 to disable.
            bass_amplitude:  uint = 4..=7,

            /// Lower limit frequency in 10Hz steps.
            /// Range: 2..=15
            ///
            /// Typically should be set to the lowest frequency that the audio
            /// system can reproduce.
            bass_bottom_frequency: uint = 0..=3,
        },
        /// Register responsible for controlling the clock settings
        register Clockf {
            const ADDRESS = 0x3;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// CLKI multiplier.
            ///
            /// CLKI = XTALI * (1 + (multiplier/2)).
            ///
            /// I.e. 0 = 1.0x, 7 = 4.5x
            multiplier: uint = 13..=15,

            /// Maximum extra multiplier for WMA decoding in 0.5x steps.
            ///
            /// 0 - no extra multiplier, 3 - up to +1.5x
            allowed_addition: uint = 11..=12,

            /// Set XTALI frequency input.
            ///
            /// Set to:
            ///
            /// input_frequency = (XTALI - 8_000_000) / 4_000
            ///
            /// where XTALI - clock frequency in Hz.
            /// Default value 0 is a synonym for the default 12.228 MHz
            input_frequency: uint = 0..=10,
        },
        /// The current decode time in full seconds.
        ///
        /// The user may change the value of this register.
        /// In that case the new value should be written twice.
        register DecodeTime {
            const ADDRESS = 0x4;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// The current decode time in seconds.
            value: uint = 0..16,
        },
        /// Contains information about the sample rate and number of channels.
        ///
        /// Can be written to overwrite the information.
        register Audata {
            const ADDRESS = 0x5;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// If the sample rate is even: true = stereo, false = mono
            /// If the sample rate is odd: true = mono, false = stereo
            ///
            /// This is due to a bug in the system firmware in VS1003b.
            stereo: bool = 0,
            sample_rate: uint = 1..16,
        },
        /// Wram is used to upload application programs and data to instruction and data RAMs.
        /// The start address must be initialized by writing to WramAddr prior to the first write/read
        /// of Wram. As 16 bits of data can be transferred with one Wram write/read, and the
        /// instruction word is 32 bits long, two consecutive writes/reads are needed for each instruction
        /// word. The byte order is big-endian (i.e. most significant words first). After each full-word
        /// write/read, the internal pointer is autoincremented.
        register Wram {
            const ADDRESS = 0x6;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;

            value: uint = 0..16,
        },
        /// WramAddr is used to set the program address for following Wram writes/reads.
        /// Address offset of 0 is used for X, 0x4000 for Y, and 0x8000 for instruction memory. Peripheral
        /// registers can also be accessed
        register WramAddr {
            type Access = WO;

            const ADDRESS = 0x7;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// The read/write pointer
            value: uint = 0..16,
        },
        /// Value depends on decoded file type and running application.
        /// Check the datasheet for details.
        register Hdat0 {
            type Access = RO;

            const ADDRESS = 0x8;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// The value
            value: uint = 0..16,
        },
        /// Value depends on decoded file type and running application.
        /// Check the datasheet for details.
        register Hdat1 {
            type Access = RO;

            const ADDRESS = 0x9;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// The value
            value: uint = 0..16,
        },

        /// AiAddr indicates the start address of the application code written earlier with WramAddr
        /// and Wram registers. If no application code is used, this register should not be initialized,
        /// or it should be initialized to zero. For more details, see Application Notes for VS10XX.
        register AiAddr {
            const ADDRESS = 0xA;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// The application start pointer.
            value: uint = 0..16,
        },
        /// Volume control.
        ///
        /// Setting both channels to 255 will activate analog powerdown mode.
        register Vol {
            const ADDRESS = 0xB;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;

            /// Right channel volume in -0.5 dB steps.
            /// 0 = 0 dB, 254 = -127 dB
            right: uint = 0..8,

            /// Left channel volume in -0.5 dB steps.
            /// 0 = 0 dB, 254 = -127 dB
            left: uint = 8..16,
        },
        /// For communication with user's application on the VS1003
        register AiCtrl {
            const ADDRESS = 0xC;
            const SIZE_BITS = 16;
            const ALLOW_ADDRESS_OVERLAP = true;
            const REPEAT = {
                count: 4,
                stride: 1,
            };

            /// The value
            value: uint = 0..16,
        },
    }
);
