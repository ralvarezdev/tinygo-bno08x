package tinygo_bno08x

import (
	"time"

	"machine"

	tinygoerrors "github.com/ralvarezdev/tinygo-errors"
	tinygologger "github.com/ralvarezdev/tinygo-logger"
)

type (
	// I2C is the I2C implementation of the BNO08X sensor.
	I2C struct {
		*BNO08X
		i2cBus   *machine.I2C
		address  uint16
		ps0Pin   machine.Pin
		ps1Pin   machine.Pin
		resetPin machine.Pin
	}

	// I2CPacketReader represents the Packet reader for the I2C interface.
	I2CPacketReader struct {
		i2cBus         *machine.I2C
		packetBuffer   PacketBuffer
		logger         tinygologger.Logger
		address        uint16 // I2C address of the device
		cachedHeader   PacketHeader
		isHeaderCached bool
	}

	// I2CPacketWriter represents the Packet writer for the I2C interface.
	I2CPacketWriter struct {
		i2cBus       *machine.I2C
		packetBuffer PacketBuffer
		logger       tinygologger.Logger
		address      uint16 // I2C address of the device
	}
)

var (
	// probeDeviceBuffer is a buffer used for probing the I2C device.
	probeDeviceBuffer = [1]byte{}
)

// probeDevice tries a zero-length write then a 1-byte read to confirm presence.
//
// Parameters:
//
// bus: The I2C bus to use for communication.
// address: The I2C address of the device to probe.
//
// Returns:
//
// An error if the probe fails, otherwise nil.
func probeDevice(bus *machine.I2C, address uint16) tinygoerrors.ErrorCode {
	// Zero-length write (some devices NACK this; tolerate)
	_ = bus.Tx(address, nil, nil)

	// Attempt to read 1 byte (BNO08X will usually NACK but if wiring wrong we get generic error)
	if err := bus.Tx(address, nil, probeDeviceBuffer[:]); err != nil {
		return ErrorCodeBNO08XI2CFailedToProbeDevice
	}
	return tinygoerrors.ErrorCodeNil
}

// NewI2C creates a new I2C instance for the BNO08X sensor.
//
// Parameters:
//
// i2cBus: The I2C bus to use for communication.
// sdaPin: The SDA pin for the I2C bus.
// sclPin: The SCL pin for the I2C bus.
// address: The I2C address of the BNO08X sensor.
// ps0: The PS0 pin to set the sensor to I2C mode.
// ps1: The PS1 pin to set the sensor to I2C mode.
// resetPin: The pin used to reset the BNO08X sensor.
// packetBuffer: The PacketBuffer to use for storing Packet data.
// afterResetFn: An optional function to be called after a reset.
// logger: The logger to use for logging and debugging information (optional).
// address0Pin: The pin used to set the I2C address (optional).
//
// Returns:
//
// A pointer to a new I2C instance or an error if initialization fails.
func NewI2C(
	i2cBus *machine.I2C,
	sdaPin machine.Pin,
	sclPin machine.Pin,
	address uint16,
	ps0Pin machine.Pin,
	ps1Pin machine.Pin,
	resetPin machine.Pin,
	packetBuffer PacketBuffer,
	afterResetFn func(b *BNO08X) tinygoerrors.ErrorCode,
	logger tinygologger.Logger,
	address0Pin *machine.Pin,
) (*I2C, tinygoerrors.ErrorCode) {
	// Check if the I2C bus is nil
	if i2cBus == nil {
		return nil, ErrorCodeBNO08XNilI2CBus
	}

	// Set PS0 pin to output and low
	ps0Pin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	ps0Pin.Low()

	// Set PS1 pin to output and lo2
	ps1Pin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	ps1Pin.Low()

	// Configure the I2C bus
	if err := i2cBus.Configure(
		machine.I2CConfig{
			SCL:       sclPin,
			SDA:       sdaPin,
			Frequency: I2CFrequency,
		},
	); err != nil {
		return nil, ErrorCodeBNO08XFailedToConfigureI2C
	}

	// Check if the address is the default or the alternative
	if address != I2CDefaultAddress && address != I2CAlternativeAddress {
		return nil, ErrorCodeBNO08XInvalidI2CAddress
	}

	// Set the Address0 pin based on the desired address
	if address0Pin != nil {
		address0Pin.Configure(machine.PinConfig{Mode: machine.PinOutput})
		if address == I2CAlternativeAddress {
			address0Pin.High()
		} else {
			address0Pin.Low()
		}
	}

	// Probe with retries
	isGood := false
	for i := 0; i < I2CProbeDeviceAttempts; i++ {
		if err := probeDevice(
			i2cBus,
			address,
		); err != tinygoerrors.ErrorCodeNil {
			time.Sleep(I2CProbeDeviceDelay)
			continue
		}
		isGood = true
		break
	}
	if !isGood {
		return nil, ErrorCodeBNO08XI2CFailedToProbeDeviceRepeatly
	}

	// Initialize the packet reader
	packetReader, err := newI2CPacketReader(
		i2cBus,
		address,
		packetBuffer,
		logger,
	)
	if err != tinygoerrors.ErrorCodeNil {
		return nil, ErrorCodeBNO08XFailedToCreatePacketReader
	}

	// Initialize the packet writer
	packetWriter, err := newI2CPacketWriter(
		i2cBus,
		address,
		packetBuffer,
		logger,
	)
	if err != tinygoerrors.ErrorCodeNil {
		return nil, ErrorCodeBNO08XFailedToCreatePacketWriter
	}

	// Initialize the BNO08X sensor
	bno08x, err := NewBNO08X(
		resetPin,
		packetReader,
		packetWriter,
		packetBuffer,
		I2CMode,
		afterResetFn,
		logger,
	)
	if err != tinygoerrors.ErrorCodeNil {
		return nil, err
	}

	return &I2C{
		BNO08X:  bno08x,
		i2cBus:  i2cBus,
		address: address,
		ps0Pin:  ps0Pin,
		ps1Pin:  ps1Pin,
	}, tinygoerrors.ErrorCodeNil
}

// GetBNO08X returns the BNO08X instance.
//
// Returns:
//
// The BNO08X instance.
func (i2c *I2C) GetBNO08X() *BNO08X {
	return i2c.BNO08X
}

// newI2CPacketWriter creates a new I2CPacketWriter instance.
//
// Parameters:
//
// i2cBus: The I2C bus to use for communication.
// address: The I2C address of the device to read from.
// packetBuffer: The packet buffer to use for storing Packet data.
// logger: The logger to use for logging and debugging information.
//
// Returns:
//
// A pointer to a new I2CPacketWriter instance, or an error if the packetBuffer is nil.
func newI2CPacketWriter(
	i2cBus *machine.I2C,
	address uint16,
	packetBuffer PacketBuffer,
	logger tinygologger.Logger,
) (*I2CPacketWriter, tinygoerrors.ErrorCode) {
	// Check if the I2C bus is nil
	if i2cBus == nil {
		return nil, ErrorCodeBNO08XNilI2CBus
	}

	// Check if the packetBuffer is provided
	if packetBuffer == nil {
		return nil, ErrorCodeBNO08XNilPacketBuffer
	}

	return &I2CPacketWriter{
		i2cBus:       i2cBus,
		packetBuffer: packetBuffer,
		logger:       logger,
		address:      address,
	}, tinygoerrors.ErrorCodeNil
}

// SendPacket sends a Packet over I2C.
//
// Parameters:
//
// channel: The channel number to send the Packet on.
// data: The data to send in the Packet.
//
// Returns:
//
// The sequence number of the Packet sent, or an error if sending fails.
func (pw *I2CPacketWriter) SendPacket(channel uint8, data []byte) (
	uint8,
	tinygoerrors.ErrorCode,
) {
	// Check if the data is nil
	if data == nil {
		return 0, ErrorCodeBNO08XNilPacketData
	}

	// Get channel sequence number
	sequenceNumber, errCode := pw.packetBuffer.GetChannelSequenceNumber(channel)
	if errCode != tinygoerrors.ErrorCodeNil {
		return 0, errCode
	}

	// Initialize the packet from data
	packet, errCode := NewPacketFromData(
		channel,
		sequenceNumber,
		data,
		pw.packetBuffer.GetBuffer()[:PacketHeaderLength], // Reuse header buffer
	)
	if errCode != tinygoerrors.ErrorCodeNil {
		return 0, ErrorCodeBNO08XFailedToCreatePacket
	}

	// Debug log the packet
	packet.Log(true, true, pw.logger)

	// Write to I2C
	if err := pw.i2cBus.Tx(pw.address, packet.Header.Buffer, nil); err != nil {
		return sequenceNumber, ErrorCodeBNO08XI2CFailedToWritePacketHeaderBuffer
	}
	if err := pw.i2cBus.Tx(pw.address, packet.Data, nil); err != nil {
		return sequenceNumber, ErrorCodeBNO08XI2CFailedToWritePacketPacketBuffer
	}

	// Update sequence number
	sequenceNumber, errCode = pw.packetBuffer.IncrementChannelSequenceNumber(channel)
	if errCode != tinygoerrors.ErrorCodeNil {
		return 0, errCode
	}
	return sequenceNumber, tinygoerrors.ErrorCodeNil
}

// newI2CPacketReader creates a new I2CPacketReader instance.
//
// Parameters:
//
// i2cBus: The I2C bus to use for communication.
// logger: The logger to use for logging and debugging information.
// packetBuffer: The packet buffer to use for storing Packet data.
// address: The I2C address of the device to read from.
//
// Returns:
//
// A pointer to a new I2CPacketReader instance.
func newI2CPacketReader(
	i2cBus *machine.I2C,
	address uint16,
	packetBuffer PacketBuffer,
	logger tinygologger.Logger,
) (*I2CPacketReader, tinygoerrors.ErrorCode) {
	// Check if the I2C bus is nil
	if i2cBus == nil {
		return nil, ErrorCodeBNO08XNilI2CBus
	}

	// Check if the packetBuffer is provided
	if packetBuffer == nil {
		return nil, ErrorCodeBNO08XNilPacketBuffer
	}

	return &I2CPacketReader{
		i2cBus:       i2cBus,
		logger:       logger,
		packetBuffer: packetBuffer,
		address:      address,
	}, tinygoerrors.ErrorCodeNil
}

// readHeader reads the Packet header from the I2C bus.
//
// Returns:
//
// A PacketHeader or an error if reading the header fails.
func (pr *I2CPacketReader) readHeader() (PacketHeader, tinygoerrors.ErrorCode) {
	// Check if the destination slice is nil
	packetBuffer := pr.packetBuffer.GetBuffer()
	if packetBuffer == nil {
		return PacketHeader{}, ErrorCodeBNO08XNilDestinationBuffer
	}

	// Check if start and end are within bounds
	if len(packetBuffer) < PacketHeaderLength {
		return PacketHeader{}, ErrorCodeBNO08XPacketBufferTooShortForPacketHeader
	}

	// Read the first 4 bytes from the I2C bus to get the Packet header.
	if err := pr.i2cBus.Tx(
		pr.address,
		nil,
		packetBuffer[:PacketHeaderLength],
	); err != nil {
		return PacketHeader{}, ErrorCodeBNO08XI2CFailedToReadPacketHeader
	}

	header, err := NewPacketHeaderFromBuffer(packetBuffer[:PacketHeaderLength])
	if err != tinygoerrors.ErrorCodeNil {
		return PacketHeader{}, err
	}

	// Debug log the header
	header.Log(false, pr.logger)
	return header, tinygoerrors.ErrorCodeNil
}

// nextHeader reads the next Packet header, using a cached header if available.
//
// Returns:
//
// A PacketHeader or an error if reading the header fails.
func (pr *I2CPacketReader) nextHeader() (PacketHeader, tinygoerrors.ErrorCode) {
	// Return cached header if available
	if pr.isHeaderCached {
		pr.isHeaderCached = false
		return pr.cachedHeader, tinygoerrors.ErrorCodeNil
	}

	header, err := pr.readHeader()
	if err != tinygoerrors.ErrorCodeNil {
		return PacketHeader{}, err
	}
	return header, tinygoerrors.ErrorCodeNil
}

// ReadPacket reads a Packet from the I2C bus.
//
// Returns:
//
// A Packet or an error if reading the Packet fails.
func (pr *I2CPacketReader) ReadPacket() (Packet, tinygoerrors.ErrorCode) {
	// Get next header (cached or read new)
	header, err := pr.nextHeader()
	if err != tinygoerrors.ErrorCodeNil {
		return Packet{}, err
	}

	// Validate header fields
	if header.PacketByteCount < PacketHeaderLength {
		return Packet{}, ErrorCodeBNO08XInvalidPacketSize
	}

	// Extract header fields
	packetByteCount := header.PacketByteCount

	// Skip header-only / empty packets
	if header.PacketByteCount == PacketHeaderLength || header.DataLength == 0 {
		if pr.logger != nil {
			pr.logger.WarningMessage(headerOnlyPacketMessage)
		}
		return Packet{}, ErrorCodeBNO08XNoPacketAvailable
	}

	// packetByteCount includes 4 header bytes
	dataLength := packetByteCount - PacketHeaderLength

	// Check if packet buffer is large enough
	packetBuffer := pr.packetBuffer.GetBuffer()
	if len(packetBuffer) < packetByteCount {
		return Packet{}, ErrorCodeBNO08XPacketBufferTooShortForPacket
	}

	// Preserve first 4 header bytes already read; read payload into slice after header.
	dataBuffer := packetBuffer[PacketHeaderLength : PacketHeaderLength+dataLength]
	if dataLength > 0 {
		if err := pr.i2cBus.Tx(
			pr.address,
			nil,
			dataBuffer,
		); err != nil {
			return Packet{}, ErrorCodeBNO08XI2CFailedToReadRequestedDataLength
		}
	}

	// Create a full Packet from the packet buffer
	packet, err := NewPacket(dataBuffer, header)
	if err != tinygoerrors.ErrorCodeNil {
		return Packet{}, err
	}

	// Debug log the packet
	packet.Log(false, false, pr.logger)
	return packet, tinygoerrors.ErrorCodeNil
}

// IsAvailableToRead checks if there is data ready to be read from the I2C bus.
//
// Returns:
//
// True if data is ready, false otherwise. It also checks for errors in the header.
func (pr *I2CPacketReader) IsAvailableToRead() bool {
	// Check if header is already cached
	if pr.isHeaderCached {
		return true
	}

	header, err := pr.readHeader()
	if err != tinygoerrors.ErrorCodeNil {
		return false
	}
	pr.cachedHeader = header
	pr.isHeaderCached = true
	return true
}
