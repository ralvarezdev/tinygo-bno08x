# tinygo-bno08x

[TinyGo](https://tinygo.org/) driver for the Hillcrest Laboratories / CEVA BNO080 / BNO08x IMU. It implements the SHTP packet protocol over I2C, SPI, UART and UART-RVC and exposes sensor reports such as quaternions, acceleration, gyro, magnetometer, step count and activity classification.

**Note:** This repository is archived and read-only. It is a GitHub fork of [`adafruit/Adafruit_CircuitPython_BNO08x`](https://github.com/adafruit/Adafruit_CircuitPython_BNO08x); the MIT license credits Bryan Siepert for Adafruit Industries and the repository carries the Adafruit Community Code of Conduct. The Go code is a TinyGo implementation of the driver.

## Features

- Transports `I2C`, `SPI`, `UART` and `UARTRVC` (modes `I2CMode`, `SPIMode`, `UARTMode`, `UARTRVCMode`).
- Hardware and software reset helpers.
- Readers for rotation vectors (normal, geomagnetic, game), acceleration, linear acceleration, gravity, gyro, magnetometer, raw sensor data, step count, shake, stability and activity classification.
- Feature enabling and calibration (begin, status, save).
- Allocation-conscious: byte-slice messages and `tinygoerrors.ErrorCode` instead of `error` values.

## Installation

```bash
go get github.com/ralvarezdev/tinygo-bno08x
```

Depends on `tinygo-logger`, `tinygo-buffers` and `tinygo-errors` (same author). It imports TinyGo's `machine` package, so it must be built for a TinyGo target.

## Usage

Create a transport with `NewI2C`, `NewSPI`, `NewUART` or `NewUARTRVC`, then build the sensor with `NewBNO08X`. See `i2c.go`, `spi.go`, `uart.go` and `uart_rvc.go` for each transport's parameters and wiring.

```go
func NewBNO08X(
    resetPin machine.Pin,
    packetReader PacketReader,
    packetWriter PacketWriter,
    packetBuffer PacketBuffer,
    mode Mode,
    afterResetFn func(b *BNO08X) tinygoerrors.ErrorCode,
    logger tinygologger.Logger,
) (*BNO08X, tinygoerrors.ErrorCode)
```

Then call `EnableFeature(featureID)`, call `Update()` periodically and read values with the getters (`GetQuaternion`, `GetAcceleration`, `GetGyro`, `GetMagnetic`, `GetSteps`, `GetActivityClassification`, ...). `NewDefaultPacketBuffer` provides a default `PacketBuffer`.

## Project structure

```
bno08x.go                           BNO08X type and sensor API
i2c.go spi.go uart.go uart_rvc.go   Transports
packet.go packet_buffer.go          SHTP packet, header and buffer handling
interfaces.go enums.go constants.go errors.go utils.go
```

## License

MIT License. See [LICENSE](LICENSE) (copyright 2020 Bryan Siepert for Adafruit Industries).
