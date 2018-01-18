#![feature(asm)]
#![feature(const_fn)]
#![feature(nll)]

#[macro_use]
extern crate bitflags;
extern crate embedded_hal as hal;

mod registers;
mod utils;

use registers::*;
use utils::delay_us;

use hal::blocking::spi;
use hal::digital::OutputPin;

/// These frequency register values are found using:
/// Freq = FRF * 32,000,000 / 2^19
/// FRF = Freq / 32,000,000 * 2^19
#[repr(u32)]
pub enum Frf {
    /// http://stakeholders.ofcom.org.uk/binaries/spectrum/spectrum-policy-area/spectrum-management/research-guidelines-tech-info/interface-requirements/IR_2030-june2014.pdf
    /// Limit 25mW = 14dBm ERP, no channel bw limit
    /// Either limit duty cycle to 1% or implement Directive 1999/5/EC or equiv.
    /// Freq = 865,913,993 Hz -> FRF = 14187134.8613 */
    Frf868 = 14187135,
    /// http://www.digikey.com/en/articles/techzone/2011/may/unlicensed-915-mhz-band-fits-many-applications-and-allows-higher-transmit-power
    /// Limit 4W = 36dBm, unsure of bw limit
    /// DSSS required but not FHSS
    /// Freq = 925,892,009 Hz -> FRF = 15169814.6755
    Frf915 = 15169815,
}

/// MFRC522 driver
pub struct Rfm95w<SPI, NSS> {
    spi: SPI,
    nss: NSS,
}

impl<E, NSS, SPI> Rfm95w<SPI, NSS>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    NSS: OutputPin,
{
    pub fn new(spi: SPI, nss: NSS, freq: Frf, power: u8) -> Result<Self, E> {
        let mut rfm95w = Self { spi: spi, nss: nss };

        rfm95w.init_helper()?;
        rfm95w.setfreq(freq)?;
        rfm95w.setpower(power)?;

        Ok(rfm95w)
    }

    fn init_helper(&mut self) -> Result<(), E> {
        // uint8_t RegOpMode, RegModemConfig1, RegModemConfig2;

        // Initialise SPI peripheral
        // self.spi.reset_high()
        // self.spi.disable_crc();
        // self.spi.init_master(
        //     spi::BaudRate::FpclkDiv64, // Slightly under 1MHz
        //     spi::Cpol::ClkTo0WhenIdle, // ???
        //     spi::Cpha::ClkTransition1,
        //     spi::Crcl::Bit8, // DFF/CRC length
        //     spi::BitOrder::MsbFirst // MSB first
        // );

        // Manual NSS handling:
        // self.spi.enable_software_slave_management();
        // self.spi.set_nss_high();
        self.nss.set_high();

        // self.spi.set_data_size(spi::DataSize::Bit8)
        // self.spi.set_fifo_reception_threshold_8bit(); // 8-bit rx-length

        // self.spi.enable()

        // Wait for chip to warm up
        delay_us(10_000);

        // Check we're in sleep mode
        self.setmode(Mode::Sleep)?;
        let mut reg_op_mode = OpMode::from_bits(self.readreg(Register::OpMode)?).unwrap();

        // Activate LoRa!
        reg_op_mode |= OpMode::LongRangeMode;
        self.writereg(Register::OpMode, reg_op_mode.bits())?;

        let reg_modem_config_1 =
            // Set bandwidth to 125kHz -> 0111
            ModemConfig1::Bw2 | ModemConfig1::Bw1 | ModemConfig1::Bw0 |
            // Set coding rate to 4/8 -> 100
            ModemConfig1::CodingRate2 |
            // Implicit header mode
            ModemConfig1::ImplicitHeaderModeOn;
        self.writereg(Register::ModemConfig1, reg_modem_config_1.bits())?;

        let reg_modem_config_2 =
            // Set SF9 = 256 chips/symbol
            ModemConfig2::SpreadingFactor3 |
            // Enable CRCs:
            ModemConfig2::RxPayloadCrcOn;
        self.writereg(Register::ModemConfig2, reg_modem_config_2.bits())?;

        Ok(())
    }

    /// Write the byte of data to the address
    fn writereg(&mut self, register: Register, data: u8) -> Result<(), E> {
        self.nss.set_low();

        let data = &[register as u8 | (1 << 7), data];

        self.spi.write(data)?;
        self.nss.set_high();

        Ok(())
    }

    /// Read a byte of data from the address
    fn readreg(&mut self, register: Register) -> Result<u8, E> {
        self.nss.set_low();
        let mut data = [register as u8 & !(1 << 7)];
        self.spi.transfer(&mut data)?;
        self.nss.set_high();

        Ok(data[0])
    }

    /// Bulk write to a register from a buffer
    fn bulkwrite(&mut self, register: Register, data: &[u8]) -> Result<(), E> {
        self.nss.set_low();
        self.spi.write(&[register as u8 | (1 << 7)])?;
        self.spi.write(data)?;
        self.nss.set_high();

        Ok(())
    }

    /// Bulk read from a register to a buffer
    fn bulkread(&mut self, register: Register, data: &mut [u8]) -> Result<(), E> {
        self.nss.set_low();
        data[0] = register as u8 & !(1 << 7);
        for byte in &mut data[1..] {
            *byte = 0;
        }
        self.spi.transfer(data)?;
        self.nss.set_high();

        Ok(())
    }

    fn setmode(&mut self, mode: Mode) -> Result<(), E> {
        // We read the old value, set the mode bits as appropriate then rewrite the
        // register, so we don't stomp all over the other register bits
        let old_mode = self.readreg(Register::OpMode)? & 0b11111000;
        self.writereg(Register::OpMode, old_mode | mode as u8)?;

        // Wait for new mode to take effect
        while self.readreg(Register::OpMode)? & 0b00000111 != mode as u8 {}

        Ok(())
    }

    /// Set the RFM95W centre frequency using an FRF register value
    pub fn setfreq(&mut self, frf: Frf) -> Result<(), E> {
        let frf = frf as u32;

        // Check the radio is sleeping to set the frequency/
        self.setmode(Mode::Sleep)?;

        // Write 24 bits of frequency
        self.writereg(Register::FrfMsb, (frf >> 16) as u8)?;
        self.writereg(Register::FrfMid, (frf >> 8) as u8)?;
        self.writereg(Register::FrfLsb, frf as u8)?;

        // Wake up the radio, spin up the synthesizers!
        self.setmode(Mode::Standby)?;

        Ok(())
    }

    /// Set transmit power to a dBm value from 2 to +17dBm
    pub fn setpower(&mut self, power: u8) -> Result<(), E> {
        let mut power = power;
        let mut pa_config: u8 = 0x00;

        // Force boost mode for the HopeRF module, restricts power range to
        // 2 - 17dBm (without using extra boost to 20dBm)
        if (power < 2) || (power > 17) {
            power = 2; // 2dBm sensible default
        }

        // Select boost PA
        pa_config |= PaConfig::PaSelect.bits();

        /* Actual Power = OutputPower + 2dBm, so set OutputPower=power-2 */
        pa_config |= power - 2;

        self.writereg(Register::PaConfig, pa_config)?;

        Ok(())
    }

    /// Transmit a packet length len stored in buf, optional PA_BOOST to 100mW TX
    pub fn transmit(&mut self, data: &[u8]) -> Result<(), E> {
        // Check we're in stand-by
        self.setmode(Mode::Standby)?;

        // Set packet length
        self.writereg(Register::PayloadLength, data.len() as u8)?;

        // Move to the beginning of the TX FIFO

        let old_addr = self.readreg(Register::FifoRxBaseAddr)?;
        self.writereg(Register::FifoAddrPtr, old_addr)?;

        // Fill the FIFO
        self.bulkwrite(Register::Fifo, data)?;

        // Request TX mode to initiate send
        self.setmode(Mode::Tx)?;

        // TODO: For now, block on sending
        while self.readreg(Register::OpMode)? & 0b00000111 == Mode::Tx as u8 {}

        // Clear txdone interrupt flag
        self.writereg(Register::OpMode, IrqFlags::TxDone.bits())?; // Really?

        Ok(())
    }

    /// Retrieve a received packet, into buf
    pub fn receive(&mut self, data: &mut [u8]) -> Result<(), E> {
        // Check we're in stand-by
        self.setmode(Mode::Standby)?;

        // Set packet length
        self.writereg(Register::PayloadLength, data.len() as u8)?;

        let mut valid_received = false;
        while !valid_received {
            // Set Fifo to beginning of RX buffer
            let old_addr = self.readreg(Register::FifoRxBaseAddr)?;
            self.writereg(Register::FifoAddrPtr, old_addr)?;

            // Initiate receive using mode change
            self.setmode(Mode::RxSingle)?;

            // Block until receipt or timeout
            let mut irq_flags = IrqFlags::all();
            while !irq_flags.intersects(IrqFlags::RxDone | IrqFlags::RxTimeout) {
                irq_flags = IrqFlags::from_bits(self.readreg(Register::IrqFlags)?).unwrap();
            }

            // Received if not timeout and CRC done and length correct
            valid_received = self.readreg(Register::RxNbBytes)? == data.len() as u8
                && irq_flags.intersects(IrqFlags::RxDone)
                && !irq_flags.intersects(IrqFlags::PayloadCrcError);

            // Clear IrqFlags::RxDone, IrqFlags::RxTimeout, and CRC fail interrupts
            self.writereg(
                Register::IrqFlags,
                (IrqFlags::RxDone | IrqFlags::PayloadCrcError | IrqFlags::RxTimeout).bits(),
            )?;
        }

        /* Move FIFO pointer to beginning of last packet received */
        let old_addr = self.readreg(Register::FifoRxBaseAddr)?;
        self.writereg(Register::FifoAddrPtr, old_addr)?;

        /* Read packet out of FIFO into our buffer */
        self.bulkread(Register::Fifo, data)?;

        Ok(())
    }

    /// Put module into receive mode then return
    pub fn receive_async(&mut self, len: u8) -> Result<(), E> {
        // Check we're in stand-by
        self.setmode(Mode::Standby)?;

        // Set packet length
        self.writereg(Register::PayloadLength, len)?;

        // Clear possible interrupts
        self.writereg(
            Register::IrqFlags,
            (IrqFlags::RxDone | IrqFlags::PayloadCrcError | IrqFlags::RxTimeout).bits(),
        )?;

        // Set Fifo to beginning of RX buffer
        let old_addr = self.readreg(Register::FifoRxBaseAddr)?;
        self.writereg(Register::FifoAddrPtr, old_addr)?;

        // Initiate receive using mode change
        self.setmode(Mode::RxContinuous)?;

        Ok(())
    }

    /// Attempt to retrieve a packet received in async mode.  Return success */
    pub fn packet_retrieve(&mut self, data: &mut [u8]) -> Result<bool, E> {
        let irq_flags = IrqFlags::from_bits(self.readreg(Register::IrqFlags)?).unwrap();
        let rx_len = self.readreg(Register::RxNbBytes)?;

        if irq_flags.intersects(IrqFlags::RxDone)
            && !irq_flags.intersects(IrqFlags::PayloadCrcError)
            && rx_len as usize == data.len()
        {
            // Good receive.
            // Move FIFO pointer to beginning of last packet received
            let old_addr = self.readreg(Register::FifoRxCurrentAddr)?;
            self.writereg(Register::FifoAddrPtr, old_addr)?;

            // Read packet out of FIFO into our buffer
            self.bulkread(Register::Fifo, data)?;

            // Clear IrqFlags::RxDone, IrqFlags::RxTimeout, and CRC fail interrupts
            self.writereg(
                Register::IrqFlags,
                (IrqFlags::RxDone | IrqFlags::PayloadCrcError | IrqFlags::RxTimeout).bits(),
            )?;

            Ok(true)
        } else {
            // Bad receive
            // Clear IrqFlags::RxDone, IrqFlags::RxTimeout, and CRC fail interrupts
            self.writereg(
                Register::IrqFlags,
                (IrqFlags::RxDone | IrqFlags::PayloadCrcError | IrqFlags::RxTimeout).bits(),
            )?;

            Ok(false)
        }
    }

    /// Check if a packet has been received and is waiting to be retrieved
    pub fn packet_waiting(&mut self) -> Result<bool, E> {
        Ok(IrqFlags::from_bits(self.readreg(Register::IrqFlags)?)
            .unwrap()
            .intersects(IrqFlags::RxDone))
    }

    /// Retrieve RSSI/SNR of last packet received
    pub fn getrssi(&mut self) -> Result<u8, E> {
        const DISPLAY_SNR: bool = true;

        if DISPLAY_SNR {
            let mut signed_val = self.readreg(Register::PktSnrValue)? as i16;
            // Shift it up by 128 to make all vals positive
            signed_val += 128;
            // Scale 0-255 to 0-99
            Ok((signed_val as u16 * 99 / 255) as u8)
        } else {
            // Scale 0-155 (highest observed) to 0-99%
            let rssi = self.readreg(Register::PktRssiValue)? as u16 * 99 / 155;
            if rssi > 99 {
                Ok(99)
            } else {
                Ok(rssi as u8)
            }
        }
    }
}
