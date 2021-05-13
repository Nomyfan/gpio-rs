extern crate memmap2;

use memmap2::{MmapMut, MmapOptions};
use std::{
    fs::{File, OpenOptions},
    io::{Error, ErrorKind, Read, Result, Seek, SeekFrom},
    slice,
    thread::sleep,
    time::Duration,
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
    PullOff = 0,
    PullDown = 1,
    PullUp = 2,
}

// Edge events
#[derive(PartialEq)]
enum Edge {
    NoEdge = 0b00000000,
    RiseEdge = 0b00000001,
    FallEdge = 0b00000010,
    AnyEdge = 0b00000011, // RiseEdge | FallEdge
}

// IRQs(High, Low) High -> high 32 bits of u64, Low -> low 32 bits of u64
#[derive(Copy, Clone)]
pub struct IRQs(u32, u32);

struct Gpio {
    gpio: MmapMut,
    clk: MmapMut,
    pwm: MmapMut,
    spi: MmapMut,
    intr: MmapMut,

    irqs_backup: IRQs,
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

impl Drop for Gpio {
    fn drop(&mut self) {
        enable_irqs(self.intr.as_u32_mut_slice(), &self.irqs_backup);
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

        let gpio = map_mem(&file, gpio_addr, PAGE_SIZE)?;
        let clk = map_mem(&file, clk_addr, PAGE_SIZE)?;
        let pwm = map_mem(&file, pwm_addr, PAGE_SIZE)?;
        let spi = map_mem(&file, spi_addr, PAGE_SIZE)?;
        let intr = map_mem(&file, intr_addr, PAGE_SIZE)?;
        let irqs_backup = read_irqs(intr.as_u32_slice()); // back up enabled IRQs, to restore it later

        Ok(Gpio {
            gpio,
            clk,
            pwm,
            spi,
            intr,
            irqs_backup,
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

    // NOTE without root permission this changes will simply do nothing successfully
    //
    // SetFreq: Set clock speed for given pin in Clock or Pwm mode
    //
    // Param freq should be in range 4688Hz - 19.2MHz to prevent unexpected behavior,
    // however output frequency of Pwm pins can be further adjusted with SetDutyCycle.
    // So for smaller frequencies use Pwm pin with large cycle range. (Or implement custom software clock using output pin and sleep.)
    //
    // Note that some pins share the same clock source, it means that
    // changing frequency for one pin will change it also for all pins within a group.
    // The groups are:
    //   gp_clk0: pins 4, 20, 32, 34
    //   gp_clk1: pins 5, 21, 42, 44
    //   gp_clk2: pins 6 and 43
    //   pwm_clk: pins 12, 13, 18, 19, 40, 41, 45
    pub fn set_freq(&mut self, pin: Pin, freq: u32) {
        // TODO: would be nice to choose best clock source depending on target frequency, oscilator is used for now
        let source_freq = 19200000u32; // oscilator frequency
        let div_mask = 4095u32; // divi and divf have 12 bits each

        let divi = (source_freq / freq) & div_mask;
        let divf = (((source_freq % freq) << 12) / freq) & div_mask;

        let (clk_ctl_reg, clk_div_reg) = {
            let ctl = 28usize;
            let div = 28usize;
            match pin {
                4 | 20 | 32 | 34 => (ctl + 0, div + 1), // clk0
                5 | 21 | 42 | 44 => (ctl + 2, div + 3), // clk1
                6 | 43 => (ctl + 4, div + 5),           // clk2
                // pwm_clk - shared clk for both pwm channels
                12 | 13 | 40 | 41 | 45 | 18 | 19 => {
                    stop_pwm(self); // pwm clk busy wont go down without stopping pwm first
                    (ctl + 12, div + 13)
                }
                _ => return,
            }
        };

        let mash = if divi < 2 || divf == 0 { 0 } else { 1u32 << 9 };

        let password = 0x5A000000u32;
        let busy = 1u32 << 7;
        let enab = 1u32 << 4;
        let src = 1u32 << 0; // oscilator

        let clk_mem = self.clk.as_u32_mut_slice();
        clk_mem[clk_ctl_reg] = password | (clk_mem[clk_ctl_reg] & !enab); // stop gpio clock (without changing src or mash)

        while clk_mem[clk_ctl_reg] & busy != 0 {
            // ... and wait for not busy
            sleep(Duration::from_micros(10));
        }

        clk_mem[clk_ctl_reg] = password | mash | src; // set mash and source (without enabling clock)
        clk_mem[clk_div_reg] = password | (divi << 12) | divf; // set dividers

        // mash and src can not be changed in same step as enab, to prevent lock-up and glitches
        sleep(Duration::from_micros(10)); // ... so wait for them to take effect

        clk_mem[clk_ctl_reg] = password | mash | src | enab; // finally start clock

        // defer code
        start_pwm(self);
    }

    pub fn enable_irqs(&mut self, irqs: &IRQs) {
        enable_irqs(self.intr.as_u32_mut_slice(), irqs);
    }

    pub fn disable_irqs(&mut self, irqs: &IRQs) {
        disable_irqs(self.intr.as_u32_mut_slice(), irqs);
    }

    // NOTE without root permission this changes will simply do nothing successfully
    // SetDutyCycle: Set cycle length (range) and duty length (data) for Pwm pin in M/S mode
    //
    //   |<- duty ->|
    //    __________
    //  _/          \_____________/
    //   |<------- cycle -------->|
    //
    // Output frequency is computed as pwm clock frequency divided by cycle length.
    // So, to set Pwm pin to freqency 38kHz with duty cycle 1/4, use this combination:
    //
    //  pin.Pwm()
    //  pin.DutyCycle(1, 4)
    //  pin.Freq(38000*4)
    //
    // Note that some pins share common pwm channel,
    // so calling this function will set same duty cycle for all pins belonging to channel.
    // The channels are:
    //   channel 1 (pwm0) for pins 12, 18, 40
    //   channel 2 (pwm1) for pins 13, 19, 41, 45.
    pub fn set_duty_cycle(&mut self, pin: Pin, duty_len: u32, cycle_len: u32) {
        let pwm_ctl_reg = 0usize;

        // shift: offset inside ctlReg
        let (pwm_dat_reg, pwm_rng_reg, shift) = match pin {
            12 | 18 | 40 => (4usize, 5usize, 0u8),      // channel pwm0
            13 | 19 | 41 | 45 => (8usize, 9usize, 8u8), // channel pwm1
            _ => return,
        };

        let ctl_mask = 255u32; // ctl setting has 8 bits for each channel
        let pwen = 1 << 0; // enable pwm
        let msen = 1 << 7; // use M/S transition instead of pwm algorithm

        let pwm = self.pwm.as_u32_mut_slice();
        // reset settings
        pwm[pwm_ctl_reg] =
            (pwm[pwm_ctl_reg] & !(ctl_mask << shift)) | (msen << shift) | (pwen << shift);

        // set duty cycle
        pwm[pwm_dat_reg] = duty_len;
        pwm[pwm_rng_reg] = cycle_len;

        sleep(Duration::from_micros(10));
    }

    pub fn set_pull_mode(&mut self, pin: Pin, pull: Pull) {
        let is_bcm2711 = self.is_bcm2711();
        let gpio_mem = self.gpio.as_u32_mut_slice();
        set_pull_mode(gpio_mem, pin, pull, is_bcm2711);
    }

    pub fn read_pull(&self, pin: Pin) -> Option<Pull> {
        read_pull(self.gpio.as_u32_slice(), pin)
    }

    pub fn detect_edge(&mut self, pin: Pin, edge: Edge) {
        let gpio_mem = self.gpio.as_u32_mut_slice();
        let intr_mem = self.intr.as_u32_mut_slice();
        detect_edge(gpio_mem, intr_mem, pin, edge);
    }

    pub fn is_edge_detected(&mut self, pin: Pin) -> bool {
        let gpio_mem = self.gpio.as_u32_mut_slice();
        is_edge_detected(gpio_mem, pin)
    }

    pub fn is_bcm2711(&self) -> bool {
        is_bcm2711(self.gpio.as_u32_slice())
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

fn set_pull_mode(gpio_mem: &mut [u32], pin: Pin, pull: Pull, is_bcm2711: bool) {
    if is_bcm2711 {
        set_pull_mode_for_bcm2711(gpio_mem, pin, pull);
    } else {
        // Pull up/down/off register has offset 38 / 39, pull is 37
        let pull_clk_reg = pin as usize / 32 + 38;
        let pull_reg = 37;
        let shift = pin % 32;

        match pull {
            Pull::PullDown | Pull::PullUp => {
                gpio_mem[pull_reg] |= pull as u32;
            }
            Pull::PullOff => {
                gpio_mem[pull_reg] &= !3;
            }
        }

        // Wait for value to clock in, this is ugly, sorry :(
        sleep(Duration::from_micros(1));

        gpio_mem[pull_clk_reg] = 1 << shift;

        // Wait for value to clock in
        sleep(Duration::from_micros(1));

        gpio_mem[pull_reg] &= !3;
        gpio_mem[pull_clk_reg] = 0;
    }
}

fn set_pull_mode_for_bcm2711(gpio_mem: &mut [u32], pin: Pin, pull: Pull) {
    let pull_reg = GPPUPPDN0 + (pin >> 4) as usize;
    let shift = (pin & 0xf) << 1;

    let p = match pull {
        Pull::PullOff => 0,
        Pull::PullUp => 1,
        Pull::PullDown => 2,
    };

    // This is verbatim C code from raspi-gpio.c
    let pull_bits = gpio_mem[pull_reg];
    let pull_bits = pull_bits & !(3 << shift);
    let pull_bits = pull_bits | (p << shift);
    gpio_mem[pull_reg] = pull_bits;
}

fn read_pull(gpio_mem: &[u32], pin: Pin) -> Option<Pull> {
    if !is_bcm2711(gpio_mem) {
        return None;
    }

    let reg = GPPUPPDN0 + (pin >> 4) as usize;
    let bits = gpio_mem[reg] >> (((pin & 0xf) << 1) & 0x3);
    match bits {
        0 => Some(Pull::PullOff),
        1 => Some(Pull::PullUp),
        2 => Some(Pull::PullDown),
        _ => None,
    }
}

// Enable edge event detection on pin.
//
// Combine with is_edge_detected() to check whether event occured.
//
// Note that using this function might conflict with the same functionality of other gpio library.
//
// It also clears previously detected event of this pin if there was any.
//
// Note that call with RiseEdge will disable previously set FallEdge detection and vice versa.
// You have to call with AnyEdge, to enable detection for both edges.
// To disable previously enabled detection call it with NoEdge.
//
// WARNING: this might make your Pi unresponsive, if this happens, you should either run the code as root,
// or add `dtoverlay=gpio-no-irq` to `/boot/config.txt` and restart your pi
fn detect_edge(gpio_mem: &mut [u32], intr_mem: &mut [u32], pin: Pin, edge: Edge) {
    if edge != Edge::NoEdge {
        // disable GPIO event interruption to prevent freezing in some cases
        disable_irqs(intr_mem, &IRQs(1 << 20, 1 << 17));
    }

    // Rising edge detect enable register (19/20 depending on bank)
    // Falling edge detect enable register (22/23 depending on bank)
    // Event detect status register (16/17)
    let ren_reg = pin as usize / 32 + 19;
    let fen_reg = pin as usize / 32 + 22;
    let eds_reg = pin as usize / 32 + 16;

    let bit = (1 << (pin & 31)) as u32;
    let edge_bits = edge as u32;

    if (edge_bits & Edge::RiseEdge as u32) > 0 {
        gpio_mem[ren_reg] |= bit;
    } else {
        gpio_mem[ren_reg] &= !bit;
    }

    if (edge_bits & Edge::FallEdge as u32) > 0 {
        gpio_mem[fen_reg] |= bit;
    } else {
        gpio_mem[fen_reg] &= !bit;
    }

    gpio_mem[eds_reg] = bit; // to clear outdated detection
}

// Checks whether edge event occured since last call
// or since detection was enabled
//
// There is no way (yet) to handle interruption caused by edge event, you have to use polling.
//
// Event detection has to be enabled first, by pin.Detect(edge)
fn is_edge_detected(gpio_mem: &mut [u32], pin: Pin) -> bool {
    // Event detect status register (16/17)
    let eds_reg = pin as usize / 32 + 16;

    let test = gpio_mem[eds_reg] & (1 << (pin & 31));
    gpio_mem[eds_reg] = test; // set bit to clear it

    test != 0
}

// Starts pwm for both channels
fn start_pwm(gpio: &mut Gpio) {
    let pwm_ctl_reg = 0;
    let pwen = 1;
    gpio.pwm.as_u32_mut_slice()[pwm_ctl_reg] &= !(pwen << 8 | pwen);
}

// Stop pwm for both channels
fn stop_pwm(gpio: &mut Gpio) {
    let pwm_ctl_reg = 0;
    let pwen = 1;
    gpio.pwm.as_u32_mut_slice()[pwm_ctl_reg] |= pwen << 8 | pwen;
}

// Enables given IRQs (by setting bit to 1 at intended position).
// See 'ARM peripherals interrupts table' in pheripherals datasheet.
// WARNING: you can corrupt your system, only use this if you know what you are doing.
fn enable_irqs(intr: &mut [u32], irqs: &IRQs) {
    let irq_enable_0 = 0x214usize / 4;
    let irq_enable_1 = 0x210usize / 4;

    intr[irq_enable_0] = irqs.0; // IRQ 32..63
    intr[irq_enable_1] = irqs.1; // IRQ 0..31
}

// Disables given IRQs (by setting bit to 1 at intended position).
// See 'ARM peripherals interrupts table' in pheripherals datasheet.
// WARNING: you can corrupt your system, only use this if you know what you are doing.
fn disable_irqs(intr: &mut [u32], irqs: &IRQs) {
    let irq_disable_0 = 0x220usize / 4;
    let irq_disable_1 = 0x21Cusize / 4;

    intr[irq_disable_0] = irqs.0; // IRQ 32..63
    intr[irq_disable_1] = irqs.1; // IRQ 0..31
}

fn read_irqs(intr: &[u32]) -> IRQs {
    let irq_enable_0 = 0x214usize / 4;
    let irq_enable_1 = 0x210usize / 4;

    IRQs(intr[irq_enable_0], intr[irq_enable_1])
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

// The Pi 4 uses a BCM 2711, which has different register offsets and base addresses than the rest of the Pi family (so far).  This
// helper function checks if we're on a 2711 and hence a Pi 4
fn is_bcm2711(gpio_mem: &[u32]) -> bool {
    gpio_mem[GPPUPPDN3] != 0x6770696f
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
