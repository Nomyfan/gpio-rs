extern crate memmap2;

use memmap2::{MmapMut, MmapOptions};
use std::{
    fs::{File, OpenOptions},
    io::{Error, ErrorKind, Read, Result, Seek, SeekFrom},
    slice,
};

// Memory offsets for gpio, see the spec for more details
const BCM_2835_BASE: usize = 0x20000000;
const GPIO_OFFSET: usize = 0x200000;
const CLK_OFFSET: usize = 0x101000;
const PWM_OFFSET: usize = 0x20C000;
const SPI_OFFSET: usize = 0x204000;
const INTR_OFFSET: usize = 0x00B000;
const PAGE_SIZE: usize = 4096;

// BCM 2711 has a different mechanism for pull-up/pull-down/enabl
const GPPUPPDN0: usize = 57;
const GPPUPPDN1: usize = 58;
const GPPUPPDN2: usize = 59;
const GPPUPPDN3: usize = 60;

enum Endianness {
    BigEndian,
    LittleEndian,
}

// Pin mode, a pin can be set in Input or Output, Clock or Pwm mode
enum PinMode {
    Input,
    Ouput,
    Clock,
    Pwm,
    Spi,
    Alt0,
    Alt1,
    Alt2,
    Alt3,
    Alt4,
    Alt5,
}
pub type Pin = u8;

// State of pin, High / Low
enum PinState {
    Low,
    High,
}

// Pull Up / Down / Off
enum Pull {
    PullOff,
    PullDown,
    PullUp,
    PullNone,
}

// Edge events
enum Edge {
    NoEdge = 0b00000000,
    RiseEdge = 0b00000001,
    FallEdge = 0b00000010,
    AnyEdge = 0b00000011, // RiseEdge | FallEdge
}

struct Gpio {
    gpio: MmapMut,
    clk: MmapMut,
    pwm: MmapMut,
    spi: MmapMut,
    intr: MmapMut,

    irps_backup: usize,
}

trait U32Slice {
    fn as_u32_slice<'a>(&'a self) -> &'a [u32];
    fn as_u32_mut_slice<'a>(&'a mut self) -> &'a mut [u32];
}

impl U32Slice for MmapMut {
    fn as_u32_slice<'a>(&'a self) -> &'a [u32] {
        let u8_slice = &self[..];
        unsafe { slice::from_raw_parts(u8_slice.as_ptr() as *const u32, u8_slice.len() / 4) }
    }

    fn as_u32_mut_slice<'a>(&'a mut self) -> &'a mut [u32] {
        let u8_slice = &mut self[..];
        unsafe { slice::from_raw_parts_mut(u8_slice.as_mut_ptr() as *mut u32, u8_slice.len() / 4) }
    }
}

impl Gpio {
    pub fn new() -> Result<Gpio> {
        let base_addr = get_base_addr();
        let gpio_addr = base_addr + GPIO_OFFSET;
        let clk_addr = base_addr + CLK_OFFSET;
        let pwm_addr = base_addr + PWM_OFFSET;
        let spi_addr = base_addr + SPI_OFFSET;
        let intr_addr = base_addr + INTR_OFFSET;

        let file = open_mem_file()?;

        Ok(Gpio {
            gpio: map_mem(&file, gpio_addr, PAGE_SIZE)?,
            clk: map_mem(&file, clk_addr, PAGE_SIZE)?,
            pwm: map_mem(&file, pwm_addr, PAGE_SIZE)?,
            spi: map_mem(&file, spi_addr, PAGE_SIZE)?,
            intr: map_mem(&file, intr_addr, PAGE_SIZE)?,
            // TODO
            irps_backup: 0x0,
        })
    }

    // Sets the mode of a given pin (Input, Output, Clock, Pwm or Spi)
    //
    // Clock is possible only for pins 4, 5, 6, 20, 21.
    // Pwm is possible only for pins 12, 13, 18, 19.
    //
    // Spi mode should not be set by this directly, use SpiBegin instead.
    pub fn set_pin_mode(&mut self, pin: Pin, mode: PinMode) {
        let fsel_reg = pin as usize / 10;
        let shift = pin % 10 * 3;

        let _in = 0b0;
        let _out = 0b1;
        let _alt0 = 0b100;
        let _alt1 = 0b101;
        let _alt2 = 0b110;
        let _alt3 = 0b111;
        let _alt4 = 0b011;
        let _alt5 = 0b010;

        let f = match mode {
            PinMode::Input => _in,
            PinMode::Ouput => _out,
            PinMode::Clock => match pin {
                4 | 5 | 6 | 32 | 34 | 42 | 43 | 44 => _alt0,
                20 | 21 => _alt5,
                _ => return,
            },
            PinMode::Pwm => match pin {
                12 | 13 | 40 | 41 | 45 => _alt0,
                18 | 19 => _alt5,
                _ => return,
            },
            PinMode::Spi => match pin {
                7 | 8 | 9 | 10 | 11 => _alt0,    // SPI0,
                35 | 36 | 37 | 38 | 39 => _alt0, // SPI0
                16 | 17 | 18 | 19 | 20 | 21 => _alt4,
                40 | 41 | 42 | 43 | 44 | 45 => _alt4,
                _ => return,
            },
            PinMode::Alt0 => _alt0,
            PinMode::Alt1 => _alt1,
            PinMode::Alt2 => _alt2,
            PinMode::Alt3 => _alt3,
            PinMode::Alt4 => _alt4,
            PinMode::Alt5 => _alt5,
        };

        let pin_mask = 7; // 111 - pinmode is 3 bits

        let mem = self.gpio.as_u32_mut_slice();
        mem[fsel_reg] = (mem[fsel_reg] & !(pin_mask << shift)) | (f << shift);
    }

    pub fn write_pin(&mut self, pin: Pin, state: PinState) {
        // Set register, 7 / 8 depending on bank
        // Clear register, 10 / 11 depending on bank
        let set_reg = pin as usize / 32 + 7;
        let clear_reg = pin as usize / 32 + 10;

        match state {
            PinState::Low => {
                self.gpio.as_u32_mut_slice()[clear_reg] = 1 << (pin & 31);
            }
            PinState::High => {
                self.gpio.as_u32_mut_slice()[set_reg] = 1 << (pin & 31);
            }
        }
    }

    pub fn toggle_pin(&mut self, pin: Pin) {
        let pin_usize = pin as usize;

        let set_reg = pin_usize / 32 + 7;
        let clear_reg = pin_usize / 32 + 10;
        let level_reg = pin_usize / 32 + 13;

        let bit: u32 = 1 << (pin & 31);

        let mem = self.gpio.as_u32_mut_slice();
        if (mem[level_reg] & bit) != 0 {
            mem[clear_reg] = bit;
        } else {
            mem[set_reg] = bit;
        }
    }

    pub fn read_pin(&self, pin: Pin) -> PinState {
        let level_reg = pin as usize / 32 + 13;

        if (self.gpio.as_u32_slice()[level_reg] & (1 << (pin & 31))) != 0 {
            return PinState::High;
        }

        PinState::Low
    }

    // The Pi 4 uses a BCM 2711, which has different register offsets and base addresses than the rest of the Pi family (so far).  This
    // helper function checks if we're on a 2711 and hence a Pi 4
    fn is_bcm2711(&self) -> bool {
        self.gpio.as_u32_slice()[GPPUPPDN3] != 0x6770696f
    }
}

fn open_mem_file() -> Result<File> {
    // Open fd for rw mem access; try dev/mem first (need root)
    let file = OpenOptions::new().read(true).write(true).open("/dev/mem");
    match file {
        Ok(file) => Ok(file),
        Err(err) => match err.kind() {
            // try gpiomem otherwise (some extra functions like clock and pwm setting wont work)
            ErrorKind::PermissionDenied => OpenOptions::new()
                .read(true)
                .write(true)
                .open("/dev/gpiomem"),
            _ => Err(err),
        },
    }
}

fn map_mem(file: &File, offset: usize, len: usize) -> Result<MmapMut> {
    let mut options = MmapOptions::new();
    options.offset(offset as u64).len(len);
    let mmap = unsafe { options.map_mut(file)? };

    Ok(mmap)
}

// Read /proc/device-tree/soc/ranges and determine the base address.
// Use the default Raspberry Pi 1 base address if this fails.
fn read_base_addr(offset: usize) -> Result<usize> {
    let mut ranges = File::open("/proc/device-tree/soc/ranges")?;

    ranges.seek(SeekFrom::Start(offset as u64))?;
    let mut buf = vec![0u8; 4];
    let read = ranges.read(&mut buf[..])?;
    if read != buf.len() {
        return Err(Error::new(ErrorKind::UnexpectedEof, "EOF"));
    }

    match read_bigendian_u32(&buf) {
        Some(out) => match out {
            0 => Err(Error::new(
                ErrorKind::InvalidData,
                "Invalid bigendian uint32",
            )),
            x => Ok(x as usize),
        },
        None => Err(Error::new(
            ErrorKind::InvalidData,
            "Cannot read as bigendian uint32",
        )),
    }
}

#[inline]
fn read_bigendian_u32(buf: &[u8]) -> Option<u32> {
    read_endian_u32(buf, Endianness::BigEndian)
}

fn read_endian_u32(buf: &[u8], endianess: Endianness) -> Option<u32> {
    match buf.len() {
        x if x < 4 => None,
        _ => {
            let b0 = buf[0] as u32;
            let b1 = buf[1] as u32;
            let b2 = buf[2] as u32;
            let b3 = buf[3] as u32;

            match endianess {
                Endianness::BigEndian => Some((b0 << 24) | (b1 << 16) | (b2 << 8) | b3),
                Endianness::LittleEndian => Some((b3 << 24) | (b2 << 16) | (b1 << 8) | b0),
            }
        }
    }
}

fn get_base_addr() -> usize {
    // Pi 2 & 3 GPIO base address is at offset 4
    match read_base_addr(4) {
        Ok(b) => b,
        // Pi 4 GPIO base address is as offset 8
        Err(..) => match read_base_addr(8) {
            Ok(b) => b,
            // Default to Pi 1
            Err(..) => BCM_2835_BASE,
        },
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_new_gpio() {
        let gpip = crate::Gpio::new();
        assert_eq!(true, gpip.is_ok());
    }

    #[test]
    fn test_is_bcm2711() {
        let gpio = crate::Gpio::new();
        assert_eq!(true, gpio.is_ok());

        let gpio = gpio.unwrap();

        #[cfg(not(feature = "pi4"))]
        let expected = false;
        #[cfg(feature = "pi4")]
        let expected = true;

        assert_eq!(expected, gpio.is_bcm2711());
    }

    #[test]
    fn test_read_endian_u32() {
        let data = vec![0x01, 0x02, 0x03, 0x04];

        let expected_be_u32 = 0x01020304;
        let expected_le_u32 = 0x04030201;

        let actual_le_u32 = crate::read_endian_u32(&data, crate::Endianness::LittleEndian);
        let actual_be_u32 = crate::read_endian_u32(&data, crate::Endianness::BigEndian);

        assert!(actual_be_u32.is_some());
        assert_eq!(expected_be_u32, actual_be_u32.unwrap());

        assert!(actual_le_u32.is_some());
        assert_eq!(expected_le_u32, actual_le_u32.unwrap());
    }
}
