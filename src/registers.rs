#![allow(non_upper_case_globals)]
#![allow(dead_code)]

/// RFM Modes, see datasheet p31
#[repr(u32)]
#[derive(Copy, Clone)]
pub enum Mode {
    Sleep = 0x00,
    Standby = 0x01,
    Tx = 0x03,
    RxContinuous = 0x05,
    RxSingle = 0x06,
    Cad = 0x07,
}

/// Selected RFM registers, all LoRa mode}
/// Use Semtech datasheet:}
/// http://www.semtech.com/images/datasheet/sx1276_77_78_79.pdf}
/// Compact list pp.90-92, details pp.108-115.
#[repr(u8)]
pub enum Register {
    /// RX+TX FIFO access register
    Fifo = 0x00,
    /// Operation mode
    OpMode = 0x01,
    /// Carrier frequency, most significant byte
    FrfMsb = 0x06,
    /// Carrier frequency, middle byte
    FrfMid = 0x07,
    /// Carrier freq., least significant byte
    FrfLsb = 0x08,
    /// PA and power control
    PaConfig = 0x09,
    /// PA current limit config, for boost
    Ocp = 0x0b,
    /// RX+TX FIFO access pointer
    FifoAddrPtr = 0x0d,
    /// TX FIFO base addr in buffer
    FifoTxBaseAddr = 0x0e,
    /// RX FIFO base addr in buffer
    FifoRxBaseAddr = 0x0f,
    /// Start address of last packet received
    FifoRxCurrentAddr = 0x10,
    /// Interrupt indicator flags
    IrqFlags = 0x12,
    /// Number of bytes received
    RxNbBytes = 0x13,
    /// Estimated last packet SNR
    PktSnrValue = 0x19,
    /// RSSI of last packet
    PktRssiValue = 0x1a,
    /// FHSS start channel
    HopChannel = 0x1c,
    /// Modem PHY config 1
    ModemConfig1 = 0x1d,
    /// Modem PHY config 2
    ModemConfig2 = 0x1e,
    /// RX timeout, only byte
    SymbTimeoutLsb = 0x1f,
    /// Preamble length, most significant byte
    PreambleMsb = 0x20,
    /// Preamble length, least significant byte
    PreambleLsb = 0x21,
    /// payload length
    PayloadLength = 0x22,
    /// maximum payload length
    MaxPayloadLength = 0x23,
    /// FHSS hop period
    HopPeriod = 0x24,
    /// Address in FIFO of last byte received
    FifoRxByteAddr = 0x25,
    /// Modem PHY config 3
    ModemConfig3 = 0x26,
    /// LoRa detection settings
    DetectOptimize = 0x31,
    /// = 0x0a for SF7-12 = 0x0c for SF 6
    DetectionThreshold = 0x37,
    /// DIO0-3 mapping
    DioMapping1 = 0x40,
    /// DIO4-5 mapping
    DioMapping2 = 0x41,
    /// RFM silicon version/revision
    Version = 0x42,
    /// High power PA settings
    PaDac = 0x4d,
}

// Selected contents of selected bitfield registers above
bitflags! {
    /// OpMode = 0x01
    pub struct OpMode: u8 {
        /// 1=LoRa, 0=FSK/OOK, only in sleep
        const LongRangeMode    = 1 << 7;
        /// Access FSK regs 0d:3f in LoRa mode
        const AccessSharedReg  = 1 << 6;
        /// Transceiver modes: 0=sleep,1=stby
        const Mode2            = 1 << 2;
        /// 2=FSTx, 3=Tx, 4=FSRx, 5=Rx
        const Mode1            = 1 << 1;
        /// Others reserved
        const Mode0            = 1 << 0;
    }
}

bitflags!{
    /// PaConfig = 0x09
    pub struct PaConfig: u8 {
        /// 1=PA_BOOST-20dBm 0=RFO-14dBm
        const PaSelect         = 1 << 7;
        /// Max Power
        const MaxPower2        = 1 << 6;
        /// Pmax = 10.8 + 0.6*MaxPower [dBm]
        const MaxPower1        = 1 << 5;
        const MaxPower0        = 1 << 4;
        /// Output power
        const OutputPower3     = 1 << 3;
        /// RFO: Pout = Pmax - 15 + OutputPower
        const OutputPower2     = 1 << 2;
        /// PA_BOOST: Pout = 2 + OutputPower
        const OutputPower1     = 1 << 1;
        const OutputPower0     = 1 << 0;
    }
}

bitflags!{
    /// IrqFlags = 0x12
    pub struct IrqFlags: u8 {
        /// RX timeout
        const RxTimeout        = 1 << 7;
        /// Packet received
        const RxDone           = 1 << 6;
        /// Payload CRC fail
        const PayloadCrcError  = 1 << 5;
        /// Valid header received
        const ValidHeader      = 1 << 4;
        /// Finished transmitting
        const TxDone           = 1 << 3;
        /// CAD timed out
        const CadDone          = 1 << 2;
        /// FHSS hop time!
        const FhssChangeChannel  = 1 << 1;
        /// Signal detected during CAD
        const CadDetected      = 1 << 0;
    }
}

bitflags!{
    /// ModemConfig1 = 0x1d
    pub struct ModemConfig1: u8 {
        /// Signal bandwidth
        const Bw3              = 1 << 7;
        /// 0000 - 1001
        const Bw2              = 1 << 6;
        /// See Semtech datasheet p112
        const Bw1              = 1 << 5;
        const Bw0              = 1 << 4;
        /// Coding rate
        const CodingRate2      = 1 << 3;
        /// 001 - 100
        const CodingRate1      = 1 << 2;
        /// See Semtech datasheet p112
        const CodingRate0      = 1 << 1;
        /// Implicit header mode, 1=on 0=off
        const ImplicitHeaderModeOn  = 1 << 0;
    }
}

bitflags!{
    /// ModemConfig2 = 0x1e
    pub struct ModemConfig2: u8 {
        /// Spreading factor
        const SpreadingFactor3  = 1 << 7;
        /// 6-12
        const SpreadingFactor2  = 1 << 6;
        /// See Semtech datasheet p113
        const SpreadingFactor1  = 1 << 5;
        const SpreadingFactor0  = 1 << 4;
        /// 0=single packet 1=continuous
        const TxContinuousMode  = 1 << 3;
        /// Payload CRC checking on receive
        const RxPayloadCrcOn   = 1 << 2;
        /// RX timeout most significant bit
        const SymbTimeout9     = 1 << 1;
        const SymbTimeout8     = 1 << 0;
    }
}

bitflags!{
    /// ModemConfig3 = 0x26
    pub struct ModemConfig3: u8 {
        /// Enable when symbol length >16ms
        const LowDataRateOptimize  = 1 << 3;
        /// Enable AGC or manual LnaGain
        const AgcAutoOn        = 1 << 2;
    }
}

bitflags!{
    /// DetectOptimize = 0x31
    pub struct DetectOptimize: u8 {
        /// LoRa detection optimize
        const DetectionOptimize2  = 1 << 2;
        /// Set to = 0x03 for SF 7-12
        const DetectionOptimize1  = 1 << 1;
        /// or = 0x05 for SF6
        const DetectionOptimize0  = 1 << 0;
    }
}
