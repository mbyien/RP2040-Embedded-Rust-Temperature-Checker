// main.rs
#![no_std]
#![no_main]

use panic_halt as _;
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    entry,
    pac,
    sio::Sio,
    watchdog::Watchdog,
    gpio::{DynPin, FunctionSio, Pin, PinId, PullType, SioInput, SioOutput},
    usb::UsbBus,
    Timer,
};

use cortex_m::delay::Delay;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_time::duration::Extensions;
use dht_sensor::{dht22, DhtReading};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

// DHT22 driver implementation
pub struct Dht22<P> {
    pin: P,
    delay: Delay,
}

impl<P: OutputPin + InputPin> Dht22<P> {
    pub fn new(pin: P, delay: Delay) -> Self {
        Self { pin, delay }
    }

    pub fn read(&mut self) -> Result<DhtReading, &'static str> {
        // Start signal - pull low for 1ms
        self.pin.set_low().map_err(|_| "Failed to set pin low")?;
        self.delay.delay_ms(1u32);
        
        // Pull high and switch to input
        self.pin.set_high().map_err(|_| "Failed to set pin high")?;
        
        // Wait for DHT22 response (80us low + 80us high)
        let mut timeout = 1000;
        while self.pin.is_high().unwrap_or(true) && timeout > 0 {
            self.delay.delay_us(1u32);
            timeout -= 1;
        }
        if timeout == 0 {
            return Err("Timeout waiting for DHT22 response");
        }

        timeout = 1000;
        while self.pin.is_low().unwrap_or(false) && timeout > 0 {
            self.delay.delay_us(1u32);
            timeout -= 1;
        }
        if timeout == 0 {
            return Err("Timeout waiting for DHT22 ready");
        }

        timeout = 1000;
        while self.pin.is_high().unwrap_or(true) && timeout > 0 {
            self.delay.delay_us(1u32);
            timeout -= 1;
        }
        if timeout == 0 {
            return Err("Timeout waiting for DHT22 start");
        }

        // Read 40 bits of data
        let mut data = [0u8; 5];
        for byte_idx in 0..5 {
            for bit_idx in 0..8 {
                // Wait for start of bit (50us low)
                timeout = 100;
                while self.pin.is_low().unwrap_or(false) && timeout > 0 {
                    self.delay.delay_us(1u32);
                    timeout -= 1;
                }
                if timeout == 0 {
                    return Err("Timeout reading bit start");
                }

                // Measure high duration to determine bit value
                let mut high_duration = 0;
                timeout = 100;
                while self.pin.is_high().unwrap_or(true) && timeout > 0 {
                    self.delay.delay_us(1u32);
                    high_duration += 1;
                    timeout -= 1;
                }

                // If high duration > 40us, it's a '1', otherwise '0'
                if high_duration > 40 {
                    data[byte_idx] |= 1 << (7 - bit_idx);
                }
            }
        }

        // Verify checksum
        let checksum = data[0].wrapping_add(data[1]).wrapping_add(data[2]).wrapping_add(data[3]);
        if checksum != data[4] {
            return Err("Checksum mismatch");
        }

        // Convert to temperature and humidity
        let humidity = ((data[0] as u16) << 8 | data[1] as u16) as f32 / 10.0;
        let mut temperature = (((data[2] & 0x7F) as u16) << 8 | data[3] as u16) as f32 / 10.0;
        
        // Handle negative temperature
        if data[2] & 0x80 != 0 {
            temperature = -temperature;
        }

        Ok(DhtReading {
            temperature,
            humidity,
        })
    }
}

// USB Thermal Printer interface
pub struct ThermalPrinter<'a> {
    serial: SerialPort<'a, UsbBus>,
}

impl<'a> ThermalPrinter<'a> {
    pub fn new(serial: SerialPort<'a, UsbBus>) -> Self {
        Self { serial }
    }

    pub fn init(&mut self) -> Result<(), usb_device::UsbError> {
        // Initialize thermal printer with ESC/POS commands
        self.write_bytes(&[0x1B, 0x40])?; // ESC @ - Initialize printer
        self.write_bytes(&[0x1B, 0x45, 0x01])?; // ESC E - Bold text
        Ok(())
    }

    pub fn print_temperature(&mut self, temp: f32, humidity: f32) -> Result<(), usb_device::UsbError> {
        let temp_str = format_temp(temp);
        let humidity_str = format_humidity(humidity);
        let timestamp = get_timestamp();

        // Print header
        self.write_str("================================\n")?;
        self.write_str("    TEMPERATURE READING\n")?;
        self.write_str("================================\n")?;
        
        // Print data
        self.write_str(&format!("Time: {}\n", timestamp))?;
        self.write_str(&format!("Temperature: {}°C\n", temp_str))?;
        self.write_str(&format!("Humidity: {}%\n", humidity_str))?;
        
        // Print footer
        self.write_str("--------------------------------\n")?;
        self.write_str("Powered by RP2040 + Rust\n")?;
        self.write_str("================================\n\n")?;
        
        // Feed paper
        self.write_bytes(&[0x0A, 0x0A, 0x0A])?; // Line feeds
        
        Ok(())
    }

    fn write_str(&mut self, s: &str) -> Result<(), usb_device::UsbError> {
        self.write_bytes(s.as_bytes())
    }

    fn write_bytes(&mut self, data: &[u8]) -> Result<(), usb_device::UsbError> {
        for chunk in data.chunks(64) {
            self.serial.write(chunk)?;
        }
        Ok(())
    }
}

// Helper functions for formatting
fn format_temp(temp: f32) -> heapless::String<16> {
    let mut s = heapless::String::new();
    if temp >= 0.0 {
        let whole = temp as u32;
        let frac = ((temp - whole as f32) * 10.0) as u32;
        write!(s, "{}.{}", whole, frac).ok();
    } else {
        let abs_temp = -temp;
        let whole = abs_temp as u32;
        let frac = ((abs_temp - whole as f32) * 10.0) as u32;
        write!(s, "-{}.{}", whole, frac).ok();
    }
    s
}

fn format_humidity(humidity: f32) -> heapless::String<16> {
    let mut s = heapless::String::new();
    let whole = humidity as u32;
    let frac = ((humidity - whole as f32) * 10.0) as u32;
    write!(s, "{}.{}", whole, frac).ok();
    s
}

fn get_timestamp() -> &'static str {
    // Simple timestamp - in real implementation, you'd use RTC
    "2024-01-01 12:00:00"
}

#[entry]
fn main() -> ! {
    // Initialize hardware
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
    ).ok().unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    // Initialize GPIO pins
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // DHT22 on GPIO 2 (configurable as input/output)
    let dht_pin = pins.gpio2.into_push_pull_output();
    let mut dht22 = Dht22::new(dht_pin, delay);

    // USB setup for thermal printer
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Raspberry Pi Foundation")
        .product("RP2040 Temperature Logger")
        .serial_number("12345678")
        .device_class(USB_CLASS_CDC)
        .build();

    let mut thermal_printer = ThermalPrinter::new(serial);
    
    // Initialize printer
    thermal_printer.init().ok();

    // Main loop
    let mut last_reading_time = timer.get_counter();
    const READING_INTERVAL: u32 = 5_000_000; // 5 seconds in microseconds

    loop {
        // Handle USB
        if usb_dev.poll(&mut [&mut thermal_printer.serial]) {
            // USB events handled automatically
        }

        // Check if it's time for a new reading
        let current_time = timer.get_counter();
        if current_time.ticks() - last_reading_time.ticks() >= READING_INTERVAL {
            match dht22.read() {
                Ok(reading) => {
                    // Print to thermal printer
                    if let Err(_) = thermal_printer.print_temperature(reading.temperature, reading.humidity) {
                        // Handle print error - maybe retry or log
                    }
                },
                Err(_) => {
                    // Handle sensor error - maybe print error message
                    thermal_printer.write_str("Error reading sensor\n\n").ok();
                }
            }
            last_reading_time = current_time;
        }

        // Small delay to prevent overwhelming the system
        delay.delay_ms(100u32);
    }
}

