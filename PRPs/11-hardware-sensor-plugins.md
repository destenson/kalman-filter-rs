# PRP: Hardware Sensor Plugin System

## Goal
Create a flexible plugin system for the Kalman filter library to interface with real hardware sensors (IMU, GPS, LiDAR, cameras) across different platforms and communication protocols, enabling real-time state estimation for robotics, drones, autonomous vehicles, and IoT applications.

## Why
- **Real-World Application**: Bridge gap between simulation and deployment
- **Platform Diversity**: Support embedded Linux, Windows, macOS, Android sensors
- **Protocol Variety**: I2C, SPI, UART, USB, Ethernet, CAN bus sensors
- **ROS Integration**: Essential for robotics community adoption
- **Market Differentiation**: Most Kalman libraries lack hardware integration

## What
Implement a trait-based sensor plugin architecture supporting:
1. Common sensor types (IMU, GPS, magnetometer, barometer, etc.)
2. Multiple communication protocols (serial, I2C, SPI, network)
3. Platform-specific APIs (Linux IIO, Windows Sensor API)
4. ROS topic subscription for sensor data
5. Real-time data streaming with buffering and timestamping

### Success Criteria
- [ ] Support 5+ common sensor types with examples
- [ ] Work on Linux, Windows, macOS platforms
- [ ] Sub-millisecond latency for local sensors
- [ ] ROS1/ROS2 integration working
- [ ] Example implementations for popular hardware
- [ ] Documentation for adding custom sensors

## All Needed Context

### Current Sensor Examples in Codebase
```yaml
- file: C:\Users\deste\repos\kalman_filter_rs\examples\sensor_fusion.rs
  why: Shows simulated GPS/IMU/Radar fusion pattern to follow
  
- file: C:\Users\deste\repos\kalman_filter_rs\examples\if_sensor_network.rs
  why: Distributed sensor network example
  
- file: C:\Users\deste\repos\kalman_filter_rs\src\types.rs
  why: NonlinearSystem trait pattern to mirror for sensors
```

### Sensor Integration References
```yaml
- url: https://docs.rs/serialport/latest/serialport/
  why: Cross-platform serial communication for UART sensors
  
- url: https://github.com/rust-embedded/linux-embedded-hal
  why: Linux I2C/SPI access for embedded sensors
  
- url: https://github.com/eldruin/lsm303agr-rs
  why: Example IMU driver implementation in Rust
  
- url: https://docs.rs/rppal/latest/rppal/
  why: Raspberry Pi GPIO/I2C/SPI for common platform
  
- url: https://github.com/adnanademovic/rosrust
  why: ROS1 client library for Rust
  
- url: https://github.com/ros2-rust/ros2_rust
  why: ROS2 client library for Rust
  
- url: https://github.com/korken89/mpu9250
  why: Popular IMU driver example

# System monitoring libraries
- url: https://docs.rs/sysinfo/latest/sysinfo/
  why: Cross-platform system information (CPU, memory, disk, network)
  
- url: https://docs.rs/nvml-wrapper/latest/nvml_wrapper/
  why: NVIDIA GPU monitoring via NVML
  
- url: https://github.com/Shnatsel/libdislocator
  why: Linux hwmon subsystem access for temperatures/voltages
  
- url: https://docs.rs/wmi/latest/wmi/
  why: Windows WMI access for system sensors
  
- url: https://github.com/OpenHardwareMonitor/OpenHardwareMonitor
  why: Reference for accessing hardware sensors on Windows
```

### Common Sensor Types and Protocols
| Sensor Type | Common Models | Protocol | Data Rate | Measurement |
|------------|---------------|----------|-----------|-------------|
| IMU | MPU9250, BNO055 | I2C/SPI | 100-1000 Hz | Accel/Gyro/Mag |
| GPS | NEO-M8N, ZED-F9P | UART/USB | 1-10 Hz | Position/Velocity |
| LiDAR | RPLidar, VLP-16 | UART/Ethernet | 10-20 Hz | Point cloud |
| Camera | USB/CSI cameras | USB/MIPI | 30-60 Hz | Images |
| Barometer | BMP280, MS5611 | I2C/SPI | 10-100 Hz | Pressure/Altitude |
| Magnetometer | HMC5883L | I2C | 10-75 Hz | Magnetic field |
| Ultrasonic | HC-SR04 | GPIO | 10-40 Hz | Distance |
| **System Sensors** | | | | |
| CPU Temperature | CPU Die sensors | SMBus/WMI | 1-10 Hz | Temperature °C |
| GPU Temperature | GPU sensors | NVML/ADL | 1-10 Hz | Temperature °C |
| Fan Speed | Motherboard/Case | SMBus/WMI | 1-10 Hz | RPM |
| Voltage Rails | VRM sensors | SMBus/WMI | 1-10 Hz | Volts |
| Power Draw | PSU/CPU/GPU | SMBus/WMI | 1-10 Hz | Watts |
| Memory Usage | System RAM | OS API | 1-100 Hz | MB/GB |
| Disk Temperature | NVMe/SATA | SMART | 0.1-1 Hz | Temperature °C |
| Network Traffic | NIC counters | OS API | 1-100 Hz | Mbps |

### Platform-Specific Considerations
```yaml
Linux:
- Industrial I/O (IIO) subsystem for kernel drivers
- /dev/i2c-*, /dev/spidev* device files
- Video4Linux2 for cameras

Windows:
- Windows Sensor API for built-in sensors
- COM ports for serial devices
- WinUSB for custom USB devices

macOS:
- IOKit for hardware access
- Core Motion for built-in sensors
- AVFoundation for cameras

Embedded:
- Direct register access via embedded-hal
- RTOS integration considerations
- DMA for high-speed sensors
```

### Timing and Synchronization
- Hardware timestamps vs system timestamps
- Clock drift compensation
- Sensor fusion time alignment
- Buffering for different sample rates
- Kalman filter prediction to common timestamp

### Known Gotchas
- Permission requirements (user in dialout/i2c groups on Linux)
- USB device enumeration varies by OS
- Real-time constraints vs Rust's safety
- Sensor calibration data storage
- Cross-compilation for embedded targets
- Thread safety for concurrent sensor access

## Implementation Blueprint

### Task List (in order)

1. **Define Sensor Trait System**
   - Create SensorPlugin trait with init/read/close
   - Define SensorMeasurement type with timestamp
   - Create SensorConfig for initialization parameters
   - Add SensorError type for plugin errors
   - Design callback system for async data

2. **Implement Core Infrastructure**
   - Create src/sensors/ module structure
   - Add sensor manager for plugin registration
   - Implement data buffering and timestamping
   - Create synchronization utilities
   - Add calibration data management

3. **Serial/UART Sensor Support**
   - Implement SerialSensor base type
   - Add NMEA parser for GPS
   - Create MAVLink parser for autopilots
   - Add ASCII protocol parser
   - Implement binary protocol support

4. **I2C/SPI Sensor Support**
   - Create I2cSensor and SpiSensor base types
   - Implement MPU9250 IMU driver
   - Add BMP280 barometer driver
   - Create magnetometer driver
   - Add Linux embedded-hal backend

5. **ROS Integration**
   - Create RosSensor plugin type
   - Implement sensor_msgs subscribers
   - Add tf2 transformation support
   - Create example launch files
   - Support both ROS1 and ROS2

6. **Platform-Specific Plugins**
   - Linux IIO sensor plugin
   - Windows Sensor API plugin
   - Android sensor plugin (via JNI)
   - macOS Core Motion plugin
   - Web browser sensor API (via WASM)

7. **System Monitoring Sensors**
   - CPU temperature and frequency monitoring
   - GPU temperature and utilization (NVIDIA/AMD)
   - Fan speed monitoring (chassis, CPU, GPU)
   - Voltage rail monitoring (VCore, DRAM, etc.)
   - Power consumption tracking (CPU, GPU, total)
   - Memory usage and bandwidth
   - Disk temperature via SMART
   - Network interface statistics

### Module Structure
```
src/
├── sensors/
│   ├── mod.rs           # Public API
│   ├── traits.rs        # Core traits
│   ├── manager.rs       # Plugin management
│   ├── buffer.rs        # Data buffering
│   ├── sync.rs          # Time synchronization
│   ├── calibration.rs   # Calibration storage
│   ├── plugins/
│   │   ├── serial.rs    # Serial/UART base
│   │   ├── i2c.rs       # I2C base
│   │   ├── spi.rs       # SPI base
│   │   ├── network.rs   # Network sensors
│   │   └── mock.rs      # Testing
│   ├── drivers/         # Specific sensors
│   │   ├── imu/
│   │   │   ├── mpu9250.rs
│   │   │   └── bno055.rs
│   │   ├── gps/
│   │   │   ├── nmea.rs
│   │   │   └── ubx.rs
│   │   ├── camera/
│   │   │   └── v4l2.rs
│   │   └── system/      # System monitoring
│   │       ├── cpu.rs   # CPU temp/freq
│   │       ├── gpu.rs   # GPU monitoring
│   │       ├── thermal.rs # Thermal zones
│   │       ├── power.rs # Power sensors
│   │       └── fans.rs  # Fan speeds
│   └── ros/
│       ├── ros1.rs
│       └── ros2.rs
```

### Trait Design
The sensor system should define a core `SensorPlugin` trait that all sensors implement. This trait should include:
- Associated types for measurements and configuration
- Methods for initialization, reading data, and querying capabilities
- Sample rate and noise characteristics for Kalman filter integration

An additional `AsyncSensorPlugin` trait should extend this for sensors that support streaming data with callbacks. All traits must be thread-safe (Send + Sync) to support concurrent sensor access.

### Cargo.toml Additions
```toml
[features]
sensors = ["serialport", "embedded-hal"]
sensors-ros = ["rosrust", "sensors"]
sensors-ros2 = ["r2r", "sensors"]
sensors-linux = ["linux-embedded-hal", "sensors"]
sensors-rpi = ["rppal", "sensors"]

[dependencies]
# Core sensor support
serialport = { version = "4.3", optional = true }
embedded-hal = { version = "1.0", optional = true }

# Platform specific
linux-embedded-hal = { version = "0.4", optional = true }
rppal = { version = "0.17", optional = true }

# ROS support
rosrust = { version = "0.9", optional = true }
r2r = { version = "0.8", optional = true }

# Protocol parsing
nmea = { version = "0.5", optional = true }
mavlink = { version = "0.12", optional = true }

[target.'cfg(target_os = "linux")'.dependencies]
# Linux specific

[target.'cfg(target_os = "windows")'.dependencies]
# Windows specific

[dev-dependencies]
# For testing with mock sensors
```

### Example Usage Patterns

#### Physical Sensors
```
// Pseudocode for using sensor plugins
use kalman_filter::sensors::{GpsSensor, ImuSensor};

let gps = GpsSensor::new("/dev/ttyUSB0", 9600)?;
let imu = ImuSensor::new_i2c("/dev/i2c-1", 0x68)?;

let mut kf = KalmanFilterBuilder::new(9, 6)  // 9D state, 6D measurement
    .build()?;

loop {
    // Read sensors
    let gps_data = gps.read()?;
    let imu_data = imu.read()?;
    
    // Predict to GPS timestamp
    kf.predict_to(gps_data.timestamp);
    
    // Update with measurements
    kf.update(&gps_data.position)?;
    kf.update(&imu_data.acceleration)?;
}
```

#### System Monitoring
```
// Pseudocode for thermal management tracking
use kalman_filter::sensors::{CpuTempSensor, GpuTempSensor, FanSensor};

// Initialize system sensors
let cpu_temp = CpuTempSensor::new()?;
let gpu_temp = GpuTempSensor::new()?;
let cpu_fan = FanSensor::new("cpu_fan")?;

// Kalman filter for thermal state estimation
// State: [cpu_temp, gpu_temp, ambient_temp, thermal_rate]
let mut thermal_kf = KalmanFilterBuilder::new(4, 3)
    .build()?;

loop {
    // Read system sensors
    let cpu_t = cpu_temp.read()?;
    let gpu_t = gpu_temp.read()?;
    let fan_rpm = cpu_fan.read()?;
    
    // Predict thermal evolution
    thermal_kf.predict();
    
    // Update with measurements
    thermal_kf.update(&[cpu_t.celsius, gpu_t.celsius, fan_rpm.rpm])?;
    
    // Use filtered state for fan control
    let estimated_ambient = thermal_kf.state()[2];
    let thermal_trend = thermal_kf.state()[3];
}
```

## Validation Gates

```bash
# Build with sensor support
cargo build --features sensors
cargo build --features sensors,sensors-ros

# Unit tests with mock sensors
cargo test --features sensors

# Integration tests (requires hardware/permissions)
sudo cargo test --features sensors -- --ignored hardware

# Test serial GPS (requires GPS connected)
sudo cargo run --example gps_tracking --features sensors

# Test I2C IMU (requires IMU on I2C bus)
sudo cargo run --example imu_fusion --features sensors-linux

# ROS integration test (requires ROS environment)
source /opt/ros/noetic/setup.bash  # or ros2 setup
cargo build --features sensors-ros
roslaunch kalman_filter sensor_test.launch
cargo run --example ros_sensor_fusion --features sensors-ros

# Cross-compilation for embedded
cargo build --features sensors --target thumbv7em-none-eabihf

# Permission check
# Linux: User must be in dialout and i2c groups
groups | grep -E "dialout|i2c" || echo "Add user to groups"

# List available sensors
cargo run --features sensors --example list_sensors

# Benchmark sensor reading performance
cargo bench --features sensors -- sensor_read

# Test on Raspberry Pi (if available)
cargo build --features sensors-rpi --target armv7-unknown-linux-gnueabihf
scp target/armv7-unknown-linux-gnueabihf/release/examples/imu_fusion pi@raspberrypi:
ssh pi@raspberrypi ./imu_fusion

# Documentation
cargo doc --features sensors,sensors-ros --open

# Check thread safety
cargo test --features sensors -- --test-threads=10

# Validate timing accuracy
cargo test --features sensors -- timing_accuracy
```

## Error Handling Strategy
- Graceful degradation when sensors unavailable
- Automatic reconnection for transient failures
- Clear permission error messages with fix instructions
- Timeout handling for unresponsive sensors
- Sensor health monitoring and diagnostics

## Security Considerations
- Validate sensor data ranges to prevent injection
- Use checksums for serial protocols
- Sanitize network sensor inputs
- Principle of least privilege for device access
- No arbitrary code execution from config files

## References for Implementation
- embedded-hal: https://docs.rs/embedded-hal/
- Linux IIO: https://www.kernel.org/doc/html/latest/driver-api/iio/
- serialport-rs examples: https://github.com/serialport/serialport-rs/tree/main/examples
- ROS sensor_msgs: http://docs.ros.org/en/api/sensor_msgs/html/
- I2C devices: https://github.com/eldruin/driver-examples
- Sensor fusion: https://github.com/jbruce12000/septic-tank/

## Notes for AI Agent
- Start with mock sensors for testing
- Implement serial GPS first (most common)
- Use embedded-hal traits for portability
- Test permissions and error handling thoroughly
- Keep sensor drivers minimal - just read data
- Let Kalman filter handle all fusion logic
- Consider async/await for streaming sensors
- Document calibration procedures clearly

## Quality Score: 7/10
Good confidence in implementation but complexity of hardware testing and platform differences add risk. Comprehensive context provided including sensor types, protocols, and platform considerations. Deductions for: hardware dependency for testing, platform-specific code complexity, and permission/security considerations. Strong foundation with trait system and clear examples.