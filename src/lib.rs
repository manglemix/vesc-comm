//! VESC communication library

// #![deny(missing_docs)]

use byteorder::{BigEndian, ByteOrder};
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

pub mod responses;

/// Connection to a VESC
pub struct VescConnection<T>(T);

impl<T: AsyncRead + Unpin + AsyncWrite> VescConnection<T> {
    /// Open a new connection with a VESC, currenly using embedded-hal Serial `Read` and `Write` traits
    pub fn new(t: T) -> Self {
        VescConnection(t)
    }

    /// Send a command over a connection, might have a response (Need to improve this)
    pub async fn get_fw_version(&mut self) -> Result<responses::FwVersion, VescErrorWithBacktrace> {
        write_packet(&[Command::FwVersion.value()], &mut self.0).await?;

        let payload = read_packet(&mut self.0).await?;

        if payload[0] != Command::FwVersion.value() {
            return Err(VescError::ParseError.into());
        }

        let mut uuid = [0u8; 12];
        for i in 0..12 {
            uuid[i] = payload[payload.len() - 12 + i];
        }

        // Longest currently defined HW_NAME is 10 characters
        // No fixed length hence ugly reading
        let mut hw = [0u8; 10];
        for i in 0..(payload.len() - 15) {
            hw[i] = payload[3 + i];
        }

        Ok(responses::FwVersion {
            major: payload[1],
            minor: payload[2],
            hw,
            uuid,
        })
    }

    /// Gets various sensor data from the VESC
    pub async fn get_values(&mut self) -> Result<responses::Values, VescErrorWithBacktrace> {
        write_packet(&[Command::GetValues.value()], &mut self.0).await?;
        self.0.read_u8().await.map_err(|_| VescError::IoError)?;

        let payload = read_packet(&mut self.0).await?;

        if payload[0] != Command::GetValues.value() {
            return Err(VescError::ParseError.into());
        }

        Ok(responses::Values {
            temp_fet: f32::from(BigEndian::read_u16(&payload[1..3])) / 10.0,
            temp_motor: f32::from(BigEndian::read_u16(&payload[3..5])) / 10.0,
            motor_current: BigEndian::read_u32(&payload[5..9]) as f32 / 100.0,
            input_current: BigEndian::read_u32(&payload[9..13]) as f32 / 100.0,
            id: BigEndian::read_u32(&payload[13..17]) as f32 / 100.0,
            iq: BigEndian::read_u32(&payload[17..21]) as f32 / 100.0,
            duty_cycle: f32::from(BigEndian::read_u16(&payload[21..23])) / 1_000.0,
            rpm: BigEndian::read_u32(&payload[23..27]) as f32,
            input_voltage: f32::from(BigEndian::read_u16(&payload[27..29])) / 10.0,
            amp_hours: BigEndian::read_u32(&payload[29..33]) as f32 / 10_000.0,
            amp_hours_charged: BigEndian::read_u32(&payload[33..37]) as f32 / 10_000.0,
            watt_hours: BigEndian::read_u32(&payload[37..41]) as f32 / 10_000.0,
            watt_hours_charged: BigEndian::read_u32(&payload[41..45]) as f32 / 10_000.0,
            tachometer: BigEndian::read_u32(&payload[45..49]),
            tachometer_abs: BigEndian::read_u32(&payload[49..53]),
            fault: responses::Fault::from_u8(payload[53]).unwrap(),
            pid_pos: BigEndian::read_u32(&payload[54..58]) as f32 / 1_000_000.0,
            controller_id: 0,
        })
    }

    /// Sets the forward current of the VESC
    pub async fn set_current(&mut self, val: u32) -> Result<(), VescErrorWithBacktrace> {
        let mut payload = [0u8; 5];
        payload[0] = Command::SetCurrent.value();

        BigEndian::write_u32(&mut payload[1..], val);

        write_packet(&payload, &mut self.0).await?;

        Ok(())
    }

    /// Sets the duty cycle in 100ths of a percent
    pub async fn set_duty(&mut self, val: u32) -> Result<(), VescErrorWithBacktrace> {
        let mut payload = [0u8; 5];
        payload[0] = Command::SetDuty.value();

        BigEndian::write_u32(&mut payload[1..], val);

        write_packet(&payload, &mut self.0).await?;

        Ok(())
    }
}

// Constructs a packet from a payload (adds start/stop bytes, length and CRC)
async fn write_packet<W: AsyncWrite + Unpin>(payload: &[u8], w: &mut W) -> Result<(), VescErrorWithBacktrace> {
    let hash = crc(&payload);

    // 2 for short packets and 3 for long packets
    w.write_all(&[0x02]).await.map_err(|_| VescError::IoError)?;

    // If payload.len() > 255, then start byte should be 3
    // and the next two should be the length
    w.write_all(&[payload.len() as u8])
        .await
        .map_err(|_| VescError::IoError)?;

    w.write_all(payload).await.map_err(|_| VescError::IoError)?;

    // Always CRC16
    w.write_all(&hash).await.map_err(|_| VescError::IoError)?;

    // Stop byte
    w.write_all(&[0x03]).await.map_err(|_| VescError::IoError)?;

    w.flush().await.map_err(|_| VescError::IoError)?;

    Ok(())
}

// Reads a packet, checks it and returns it's payload
async fn read_packet<R: AsyncRead + Unpin>(r: &mut R) -> Result<Vec<u8>, VescErrorWithBacktrace> {
    let mut payload;

    // Read correct number of bytes into payload
    {
        println!("a");
        let payload_len: usize = match r.read_u8().await.map_err(|_| VescError::IoError)? {
            0x02 => r.read_u8().await.map_err(|_| VescError::IoError)? as usize,
            0x03 => {
                let mut buf = [0u8; 2];
                r.read_exact(&mut buf)
                    .await
                    .map_err(|_| VescError::IoError)?;
                BigEndian::read_u16(&buf).into()
            }
            x => {
                println!("b {x} {}", u8::reverse_bits(x));
                return Err(VescError::IoError.into());
            }
        };

        payload = vec![0u8; payload_len];
        r.read_exact(&mut payload)
            .await
            .map_err(|_| VescError::IoError)?;
    }

    // Check CRC
    {
        let calculated_hash = crc(&payload);

        let read_hash = {
            let mut hash: [u8; 2] = [0; 2];
            r.read_exact(&mut hash)
                .await
                .map_err(|_| VescError::IoError)?;
            hash
        };

        if calculated_hash != read_hash {
            return Err(VescError::ChecksumError.into());
        }
    }

    // Sanity check that the last byte is the stop byte
    if r.read_u8().await.map_err(|_| VescError::IoError)? != 0x03 {
        return Err(VescError::ParseError.into());
    }

    Ok(payload)
}

fn crc(payload: &[u8]) -> [u8; 2] {
    let mut hash: [u8; 2] = [0; 2];
    BigEndian::write_u16(
        &mut hash,
        crc16::State::<crc16::XMODEM>::calculate(&payload),
    );

    hash
}

/// Errors returned if a command fails
#[derive(Debug)]
pub enum VescError {
    /// Error occured during IO
    IoError,
    /// Checksum mismatch
    ChecksumError,
    /// Error occured during parsing
    ParseError,
}

/// stub
#[derive(Debug)]
pub struct VescErrorWithBacktrace {
    pub error: VescError,
    pub backtrace: std::backtrace::Backtrace,
}


impl From<VescError> for VescErrorWithBacktrace {
    fn from(error: VescError) -> Self {
        Self {
            error,
            backtrace: std::backtrace::Backtrace::capture(),
        }
    }
}

impl std::fmt::Display for VescErrorWithBacktrace {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}\n\n{}", self.error, self.backtrace)
    }
}

impl std::fmt::Display for VescError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            VescError::IoError => write!(f, "Error occurred during IO"),
            VescError::ChecksumError => write!(f, "Checksum mismatch"),
            VescError::ParseError => write!(f, "Error occured during parsing"),
        }
    }
}

impl std::error::Error for VescError {}
impl std::error::Error for VescErrorWithBacktrace {}

#[allow(dead_code)]
#[derive(Debug)]
enum Command {
    FwVersion,
    JumpToBootloader,
    EraseNewApp,
    WriteNewAppData,
    GetValues,
    SetDuty,
    SetCurrent,
    SetCurrentBrake,
    SetRpm,
    SetPos,
    SetHandbrake,
    SetDetect,
    SetServoPos,
    SetMcConf,
    GetMcConf,
    GetMcConfDefault,
    SetAppConf,
    GetAppConf,
    GetAppConfDefault,
    SamplePrint,
    TerminalCmd,
    DetectMotorParam,
    DetectMotorRL,
    DetectMotorFluxLinkage,
    DetectEncoder,
    DetectHallFoc,
    Reboot,
    Alive,
    GetDecodedPpm,
    GetDecodedAdc,
    GetDecodedChuck,
    ForwardCan,
    SetChuckData,
    CustomAppData,
    NrfStartPairing,
}

impl Command {
    fn value(&self) -> u8 {
        match *self {
            Command::FwVersion => 0,
            Command::JumpToBootloader => 1,
            Command::EraseNewApp => 2,
            Command::WriteNewAppData => 3,
            Command::GetValues => 4,
            Command::SetDuty => 5,
            Command::SetCurrent => 6,
            Command::SetCurrentBrake => 7,
            Command::SetRpm => 8,
            Command::SetPos => 9,
            Command::SetHandbrake => 10,
            Command::SetDetect => 11,
            Command::SetServoPos => 12,
            Command::SetMcConf => 13,
            Command::GetMcConf => 14,
            Command::GetMcConfDefault => 15,
            Command::SetAppConf => 16,
            Command::GetAppConf => 17,
            Command::GetAppConfDefault => 18,
            Command::SamplePrint => 19,
            Command::TerminalCmd => 20,
            Command::DetectMotorParam => 21,
            Command::DetectMotorRL => 22,
            Command::DetectMotorFluxLinkage => 23,
            Command::DetectEncoder => 24,
            Command::DetectHallFoc => 25,
            Command::Reboot => 26,
            Command::Alive => 27,
            Command::GetDecodedPpm => 28,
            Command::GetDecodedAdc => 29,
            Command::GetDecodedChuck => 30,
            Command::ForwardCan => 31,
            Command::SetChuckData => 32,
            Command::CustomAppData => 33,
            Command::NrfStartPairing => 34,
        }
    }
}

impl responses::Fault {
    /*
    fn value(&self) -> u8 {
        match *self {
            responses::Fault::None => 0,
            responses::Fault::OverVoltage => 1,
            responses::Fault::UnderVoltage => 2,
            responses::Fault::Drv => 3,
            responses::Fault::AbsOverCurrent => 4,
            responses::Fault::OverTempFet => 5,
            responses::Fault::OverTempMotor => 6,
        }
    }
    */

    fn from_u8(n: u8) -> Result<Self, crate::VescErrorWithBacktrace> {
        match n {
            0 => Ok(responses::Fault::None),
            1 => Ok(responses::Fault::OverVoltage),
            2 => Ok(responses::Fault::UnderVoltage),
            3 => Ok(responses::Fault::Drv),
            4 => Ok(responses::Fault::AbsOverCurrent),
            5 => Ok(responses::Fault::OverTempFet),
            6 => Ok(responses::Fault::OverTempMotor),
            _ => Err(crate::VescError::ParseError.into()),
        }
    }
}
