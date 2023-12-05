#![no_std]
#![deny(unsafe_code)]
#![allow(non_snake_case)]

use core::fmt::Debug;
use embedded_hal::digital::{OutputPin, PinState};
use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind};
use modular_bitfield::prelude::*;


mod w25q32jv;
#[cfg(feature = "async")]
mod w25q32jv_async;

pub const PAGE_SIZE: u32 = 256;
pub const N_PAGES: u32 = 16384;
pub const CAPACITY: u32 = PAGE_SIZE * N_PAGES;

pub const SECTOR_SIZE: u32 = PAGE_SIZE * 16;
pub const N_SECTORS: u32 = N_PAGES / 16;
pub const BLOCK_32K_SIZE: u32 = SECTOR_SIZE * 8;
pub const N_BLOCKS_32K: u32 = N_SECTORS / 8;
pub const BLOCK_64K_SIZE: u32 = BLOCK_32K_SIZE * 2;
pub const N_BLOCKS_64K: u32 = N_BLOCKS_32K / 2;

/// Status and control register 1 
#[derive(Clone, Copy)]
#[bitfield]
pub struct StatusAndControlRegister1 {
    ///  (BUSY) [Read only], indicates that an erase/write is in progress 
    busy: B1,
    /// (WEL) Read only, indicates that the write enable instruction has been received and the device is ready to process write commands.
    writeEnableLatch: B1,
    /// (BP0,1,2) Read/write, Provides wirte protection control and status.
    blockProtectBits: B3,
    /// (TB) Read/write, The non-volatile Top/Bottom bit (TB) controls if the Block Protect Bits (BP2, BP1, BP0) protect from the Top (TB=0) or the Bottom (TB=1) of the array as shown in the Status Register Memory Protection table. The factory default setting is TB=0. The TB bit can be set with the Write Status Register Instruction depending on the state of the SRP/SRL and WEL bits.
    topBottomBlockProtect: B1,
    /// (SEC) Read/write, The non-volatile Sector/Block Protect bit (SEC) controls if the Block Protect Bits (BP2, BP1, BP0) protect either 4KB Sectors (SEC=1) or 64KB Blocks (SEC=0) in the Top (TB=0) or the Bottom (TB=1) of the array as shown in the Status Register Memory Protection table. The default setting is SEC=0.
    sectorBlockProtect: B1,
    /// (SRP) Read/Write 
    statusRegisterProtect: B1,
}


/// Status and control register 2
#[derive(Clone, Copy)]
#[bitfield]
pub struct StatusAndControlRegister2 {
    /// (SRL) Read/Write
    statusRegisterLock: B1,
    /// (QE) Read/Write, enables or disables Quad SPI operation of the chip
    quadEnable: B1,
    #[skip]
    __: B1,
    /// (LB1, LB2, LB3) Read/write, Provide the write protect control and status to the Security Register. 
    securityRegisterLockBits: B3,
    /// (CMP) Read/write, The Complement Protect bit (CMP) is a non-volatile read/write bit in the status register (S14). It is used in conjunction with SEC, TB, BP2, BP1 and BP0 bits to provide more flexibility for the array protection. Once CMP is set to 1, previous array protection set by SEC, TB, BP2, BP1 and BP0 will be reversed. For instance, when CMP=0, a top 64KB block can be protected while the rest of the array is not; when CMP=1, the top 64KB block will become unprotected while the rest of the array become read-only. Please refer to the Status Register Memory Protection table for details. The default setting is CMP=0
    complementProtect: B1,
    /// (SUS) Read only, Indicates if an erase or program action is suspended using the Erase/Program Suspend (75h) instruction.
    eraseProgramSuspendStatus: B1,
}

/// Status and control register 3
#[derive(Clone, Copy)]
#[bitfield]
pub struct StatusAndControlRegister3 {
    #[skip]
    __: B1,
    #[skip]
    __: B1,
    /// (WPS) Read/Write. Selects which Write Protect scheme should be used.
    writeProtectSelection: B1,
    #[skip]
    __: B1,
    #[skip]
    __: B1,
    /// (DRV0, DRV1) Read/Write. Used to determine the output driver strength for the Read operations.
    OutputDriverStrength: B2,
    #[skip]
    __: B1,
}

/// Low level driver for the w25q32jv flash memory chip.
pub struct W25q32jv<SPI, HOLD, WP> {
    spi: SPI,
    hold: HOLD,
    wp: WP,
}

impl<SPI, HOLD, WP> W25q32jv<SPI, HOLD, WP> {
    /// Get the capacity of the flash chip in bytes.
    pub fn capacity() -> usize {
        CAPACITY as usize
    }
}

impl<SPI, S: Debug, P: Debug, HOLD, WP> W25q32jv<SPI, HOLD, WP>
where
    SPI: embedded_hal::spi::ErrorType<Error = S>,
    HOLD: OutputPin<Error = P>,
    WP: OutputPin<Error = P>,
{
    pub fn new(spi: SPI, hold: HOLD, wp: WP) -> Result<Self, Error<S, P>> {
        let mut flash = W25q32jv { spi, hold, wp };

        flash.hold.set_high().map_err(Error::PinError)?;
        flash.wp.set_high().map_err(Error::PinError)?;

        Ok(flash)
    }

    /// Set the hold pin state.
    ///
    /// The driver doesn't do anything with this pin. When using the chip, make sure the hold pin is not asserted.
    /// By default this means the pin needs to be high (true).
    ///
    /// This function sets the pin directly and can cause the chip to not work.
    pub fn set_hold(&mut self, value: PinState) -> Result<(), Error<S, P>> {
        self.hold.set_state(value).map_err(Error::PinError)?;
        Ok(())
    }

    /// Set the write protect pin state.
    ///
    /// The driver doesn't do anything with this pin. When using the chip, make sure the hold pin is not asserted.
    /// By default this means the pin needs to be high (true).
    ///
    /// This function sets the pin directly and can cause the chip to not work.
    pub fn set_wp(&mut self, value: PinState) -> Result<(), Error<S, P>> {
        self.wp.set_state(value).map_err(Error::PinError)?;
        Ok(())
    }
}

impl<SPI, S: Debug, P: Debug, HOLD, WP> ErrorType for W25q32jv<SPI, HOLD, WP>
where
    SPI: embedded_hal::spi::ErrorType<Error = S>,
    HOLD: OutputPin<Error = P>,
    WP: OutputPin<Error = P>,
{
    type Error = Error<S, P>;
}

/// Custom error type for the various errors that can be thrown by W25q32jv.
/// Can be converted into a NorFlashError.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error<S: Debug, P: Debug> {
    SpiError(S),
    PinError(P),
    NotAligned,
    OutOfBounds,
    WriteEnableFail,
    ReadbackFail,
}

impl<S: Debug, P: Debug> NorFlashError for Error<S, P> {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Error::NotAligned => NorFlashErrorKind::NotAligned,
            Error::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            _ => NorFlashErrorKind::Other,
        }
    }
}

/// Easily readable representation of the command bytes used by the flash chip.
#[repr(u8)]
enum Command {
    PageProgram = 0x02,
    ReadData = 0x03,
    ReadStatusRegister1 = 0x05,
    ReadStatusRegister2 = 0x35,
    ReadStatusRegister3 = 0x15,
    WriteStatusRegister1 = 0x01,
    WriteStatusRegister2 = 0x31,
    WriteStatusRegister3 = 0x11,    
    WriteEnable = 0x06,
    SectorErase = 0x20,
    UniqueId = 0x4B,
    Block32Erase = 0x52,
    Block64Erase = 0xD8,
    ChipErase = 0xC7,
    EnableReset = 0x66,
    PowerDown = 0xB9,
    ReleasePowerDown = 0xAB,
    Reset = 0x99,
}

fn command_and_address(command: u8, address: u32) -> [u8; 4] {
    [
        command,
        // MSB, BE
        ((address & 0xFF0000) >> 16) as u8,
        ((address & 0x00FF00) >> 8) as u8,
        ((address & 0x0000FF) >> 0) as u8,
    ]
}
