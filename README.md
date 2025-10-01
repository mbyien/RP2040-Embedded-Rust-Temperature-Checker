# Embedded-Rust-Temperature-Checker-RP2040

A robust temperature logging system developed in Rust for the Raspberry Pi Pico (RP2040). The system supports multiple configurations, provides USB CDC data export fpr thermal printing (which can be configured to a .txt format), and includes comprehensive analytics for detailed monitoring and evaluation.

## Features

- 🌡️ **DHT22 Temperature & Humidity Sensing**
- 🖨️ **Text Output** with formatted receipts
- ⏰ **Multiple RTC Options** (Built-in, DS3231, Software-based)
- 🔢 **Print Counter** with automatic summaries
- 💻 **USB Serial Interface** for commands and monitoring
- 📝 **File Logging Support** (SD card ready)
- 🔄 **Configurable Reading Intervals**

## Hardware Requirements

### Basic Setup
- **Raspberry Pi Pico** (RP2040-based board)
- **DHT22** temperature/humidity sensor
- **Micro USB cable** for power and communication

### Optional Hardware
- **DS3231 RTC Module** (for persistent timekeeping)
- **SD Card Module** (for file logging)
- **Breadboard and jumper wires**

## Wiring Diagram

```
RP2040 Pico          DHT22 Sensor
├─ GPIO 2     ────── Data Pin
├─ 3.3V       ────── VCC
└─ GND        ────── GND

Optional DS3231 RTC:
├─ GPIO 4 (SDA) ──── SDA
├─ GPIO 5 (SCL) ──── SCL
├─ 3.3V         ──── VCC
└─ GND          ──── GND

USB Serial:
└─ USB Port     ──── Connect to computer/hub
```

## Software Setup

### Prerequisites
1. **Rust** (latest stable version)
2. **cargo-embed** or **probe-rs** for flashing
3. **Serial terminal** (like PuTTY, screen, or minicom)

### Installation

1. **Clone the repository:**
```bash
git clone https://github.com/yourusername/rp2040-temp-logger.git
cd rp2040-temp-logger
```

2. **Install Rust embedded tools:**
```bash
rustup target add thumbv6m-none-eabi
cargo install cargo-embed
```

3. **Configure your hardware** in `main.rs`:
   - Choose your RTC implementation (lines 400-430)
   - Set initial date/time
   - Configure pin assignments if different

4. **Build and flash:**
```bash
cargo embed --release
```

## RTC Implementation Options

### Option 1: Built-in RP2040 RTC (Default)
```rust
// Already configured in main.rs
let mut rtc = BuiltInRTC::new(pac.RTC, &mut pac.RESETS);
```
- ✅ No extra hardware needed
- ✅ Simple setup
- ❌ Loses time on power loss

### Option 2: DS3231 External RTC (Recommended)
```rust
// Uncomment DS3231 section in main.rs
let mut rtc = DS3231RTC::new(i2c);
rtc.set_datetime(2024, 8, 23, 14, 30, 0).ok();
```
- ✅ Battery backup
- ✅ High precision (±2ppm)
- ✅ Built-in temperature sensor
- ❌ Requires additional hardware

### Option 3: Software RTC
```rust
// Uncomment Software RTC section in main.rs
let rtc = SoftwareRTC::new(start_unix_timestamp, start_ticks, 1_000_000);
```
- ✅ No hardware needed
- ✅ Network time sync capable
- ❌ Resets on power cycle

## Usage

### Serial Commands
Connect to the device via USB serial (115200 baud) and use these commands:

| Command | Description |
|---------|-------------|
| `start` | Begin temperature logging |
| `stop` | Stop logging and show summary |
| `status` | Show current status and last reading time |
| `summary` | Display detailed analytics |
| `recent` | Show last 10 readings with differences |
| `help` | Show all available commands |

### Thermal Printer Output
Each reading produces a formatted receipt:
```
================================
    TEMPERATURE READING
================================
Reading #1
Time: 2024-08-23 14:30:00
Temperature: 22.5°C
Humidity: 45.0%
--------------------------------
Powered by RP2040 + Rust
RTC Status: OK
================================
```


## Configuration

### Reading Intervals
Modify in `main.rs`:
```rust
const READING_INTERVAL: u32 = 300_000_000; // 5 minutes in microseconds
```

### Print Counter Summary Frequency
```rust
const SUMMARY_INTERVAL: u32 = 10; // Every 10 readings
```

### Log Buffer Size
```rust
log_entries: Vec<TempLogEntry, 50>, // Store last 50 readings
```

## Project Structure

```
src/
├── main.rs                 # Main application logic
├── lib.rs                  # Library definitions
└── components/
    ├── dht22.rs           # DHT22 sensor driver
    ├── rtc/
    │   ├── builtin.rs     # Built-in RTC implementation
    │   ├── ds3231.rs      # DS3231 external RTC
    │   └── software.rs    # Software RTC
    ├── thermal_printer.rs  # Thermal printer interface
    ├── logger.rs          # Temperature logging system
    └── serial.rs          # USB serial interface

Cargo.toml                 # Dependencies and configuration
memory.x                   # Memory layout
.cargo/                    # Cargo configuration
├── config.toml
└── embed.toml
```

## Dependencies

Key Rust crates used:

```toml
[dependencies]
rp-pico = "0.8"                    # RP2040 HAL
cortex-m = "0.7"                   # ARM Cortex-M
embedded-hal = "0.2.5"             # Hardware abstraction
embedded-time = "0.12"             # Time handling
dht-sensor = "0.2"                 # DHT22 support
usb-device = "0.2"                 # USB functionality
usbd-serial = "0.1"                # USB serial
heapless = "0.7"                   # No-std collections
nb = "1.0"                         # Non-blocking APIs
panic-halt = "0.2"                 # Panic handler
```

## Troubleshooting

### Common Issues

**DHT22 not responding:**
- Check wiring connections
- Ensure 3.3V power supply
- Try different GPIO pin
- Add pull-up resistor (4.7kΩ) on data line

**RTC time incorrect:**
- For built-in RTC: Set time after each power cycle
- For DS3231: Check battery and I2C connections
- Verify I2C address (0x68 for DS3231)


**Serial commands not recognized:**
- Ensure 115200 baud rate
- Use line endings (CR/LF)
- Check USB drivers
- Try different terminal software

### Debug Output
Enable debug logging by modifying the panic handler:
```rust
use panic_semihosting as _; // Instead of panic_halt
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Code Style
- Use `cargo fmt` for formatting
- Run `cargo clippy` for linting
- Follow embedded Rust best practices
- Document public APIs

## Future Enhancements

- [ ] **Web Interface** via WiFi module
- [ ] **Data Visualization** with graphs
- [ ] **Alert System** for temperature thresholds
- [ ] **Multiple Sensor Support**
- [ ] **Network Time Sync** (NTP)
- [ ] **MQTT Integration**
- [ ] **Battery Power Management**
- [ ] **Enclosure Design** (3D printable)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **Raspberry Pi Foundation** for the RP2040 microcontroller
- **Rust Embedded Working Group** for excellent embedded support
- **DHT22/AM2302 sensor** manufacturers

## Support

For questions, issues, or contributions:
- 📧 **Email**: your-email@example.com
- 🐛 **Issues**: [GitHub Issues](https://github.com/yourusername/rp2040-temp-logger/issues)
- 💬 **Discussions**: [GitHub Discussions](https://github.com/yourusername/rp2040-temp-logger/discussions)

---
