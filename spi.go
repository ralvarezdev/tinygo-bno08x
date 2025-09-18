package tinygo_bno08x

import (
	"time"

	"machine"

	tinygoerrors "github.com/ralvarezdev/tinygo-errors"
	tinygologger "github.com/ralvarezdev/tinygo-logger"
)

type (
	// SPI is the SPI implementation of the BNO08X sensor
	SPI struct {
		*BNO08X
		spiBus   *machine.SPI
		ps0Pin   machine.Pin
		ps1Pin   machine.Pin
		resetPin machine.Pin
		csPin    machine.Pin
		intPin   machine.Pin
	}

	// SPIPacketReader is the packet reader for SPI interface
	SPIPacketReader struct {
		spiBus         *machine.SPI
		intPin         machine.Pin
		packetBuffer   PacketBuffer
		logger         tinygologger.Logger
		isHeaderCached bool
		cachedHeader   PacketHeader
	}

	// SPIPacketWriter is the packet writer for SPI interface
	SPIPacketWriter struct {
		spiBus       *machine.SPI
		intPin       machine.Pin
		packetBuffer PacketBuffer
		logger       tinygologger.Logger
	}
)

var (
	// waitingForINTMessage is the message printed when waiting for INT pin
	waitingForINTMessage = []byte("Waiting for INT...")
)

// NewSPI creates a new SPI instance for the BNO08X sensor
//
// Parameters:
//
// spiBus: The SPI bus to use for communication.
// sckPin: The SCK pin for SPI communication.
// mosiPin: The MOSI pin for SPI communication.
// misoPin: The MISO pin for SPI communication.
// csPin: The CS pin for SPI communication.
// intPin: The INT pin for SPI communication.
// ps0Pin: The PS0 pin to set the sensor to SPI mode.
// ps1Pin: The PS1 pin to set the sensor to SPI mode.
// resetPin: The pin used to reset the BNO08X sensor.
// packetBuffer: The packet buffer to use for storing Packet data.
// afterResetFn: An optional function to be called after a reset.
// logger: The logger to use for logging and debugging information (optional).
//
// Returns:
//
// A pointer to a new SPI instance and an error if any occurs.
func NewSPI(
	spiBus *machine.SPI,
	sckPin machine.Pin,
	mosiPin machine.Pin,
	misoPin machine.Pin,
	csPin machine.Pin,
	intPin machine.Pin,
	ps0Pin machine.Pin,
	ps1Pin machine.Pin,
	resetPin machine.Pin,
	packetBuffer PacketBuffer,
	afterResetFn func(b *BNO08X) tinygoerrors.ErrorCode,
	logger tinygologger.Logger,
) (*SPI, tinygoerrors.ErrorCode) {
	// Check if the SPI bus is nil
	if spiBus == nil {
		return nil, ErrorCodeBNO08XNilSPIBus
	}

	// Configure CS pin as output and high
	csPin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	csPin.High()

	// Configure INT pin as input
	intPin.Configure(machine.PinConfig{Mode: machine.PinInput})

	// Set PS0 pin to output and high
	ps0Pin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	ps0Pin.High()

	// Set PS1 pin to output and high
	ps1Pin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	ps1Pin.High()

	// Configure SPI
	if err := spiBus.Configure(
		machine.SPIConfig{
			Frequency: SPIFrequency,
			LSBFirst:  false,
			Mode:      SPIWireMode,
			SCK:       sckPin,
			SDO:       mosiPin,
			SDI:       misoPin,
		},
	); err != nil {
		return nil, ErrorCodeBNO08XFailedToConfigureSPI
	}

	// Create packet reader and writer
	packetReader, err := newSPIPacketReader(
		spiBus,
		intPin,
		packetBuffer,
		logger,
	)
	if err != tinygoerrors.ErrorCodeNil {
		return nil, ErrorCodeBNO08XFailedToCreatePacketReader
	}

	packetWriter, err := newSPIPacketWriter(
		spiBus,
		intPin,
		packetBuffer,
		logger,
	)
	if err != tinygoerrors.ErrorCodeNil {
		return nil, ErrorCodeBNO08XFailedToCreatePacketWriter
	}

	// Initialize BNO08X
	bno08x, err := NewBNO08X(
		resetPin,
		packetReader,
		packetWriter,
		packetBuffer,
		SPIMode,
		afterResetFn,
		logger,
	)
	if err != tinygoerrors.ErrorCodeNil {
		return nil, err
	}

	return &SPI{
		BNO08X:   bno08x,
		spiBus:   spiBus,
		ps1Pin:   ps1Pin,
		ps0Pin:   ps0Pin,
		resetPin: resetPin,
	}, tinygoerrors.ErrorCodeNil
}

// GetBNO08X returns the BNO08X instance.
//
// Returns:
//
// The BNO08X instance.
func (spi *SPI) GetBNO08X() *BNO08X {
	return spi.BNO08X
}

// waitForInt waits for the INT pin to go low, indicating data is ready.
//
// Parameters:
//
// intPin: The INT pin to monitor for data readiness.
// logger: The logger to use for logging and debugging information (optional).
//
// Returns:
//
// An error if the wait times out.
func waitForInt(
	intPin machine.Pin,
	logger tinygologger.Logger,
) tinygoerrors.ErrorCode {
	if logger != nil {
		logger.InfoMessage(waitingForINTMessage)
	}

	startTime := time.Now()
	for time.Since(startTime) < SPIIntTimeout {
		if !intPin.Get() {
			break
		}
	}
	return ErrorCodeBNO08XFailedToWakeUpSPI
}

// newSPIPacketReader creates a new SPIPacketReader instance.
//
// Parameters:
//
// spiBus: The SPI bus to use for communication.
// intPin: The INT pin to monitor for data readiness.
// logger: The logger to use for logging and debugging information.
// packetBuffer: The packet buffer to use for storing Packet data.
//
// Returns:
//
// A pointer to a new SPIPacketReader instance.
func newSPIPacketReader(
	spiBus *machine.SPI,
	intPin machine.Pin,
	packetBuffer PacketBuffer,
	logger tinygologger.Logger,
) (*SPIPacketReader, tinygoerrors.ErrorCode) {
	// Check if the SPI bus is nil
	if spiBus == nil {
		return nil, ErrorCodeBNO08XNilSPIBus
	}

	// Check if the packetBuffer is provided
	if packetBuffer == nil {
		return nil, ErrorCodeBNO08XNilPacketBuffer
	}

	return &SPIPacketReader{
		spiBus:       spiBus,
		intPin:       intPin,
		logger:       logger,
		packetBuffer: packetBuffer,
	}, tinygoerrors.ErrorCodeNil
}

// waitForInt waits for the INT pin to go low, indicating data is ready.
//
// Returns:
//
// An error if the wait times out.
func (pr *SPIPacketReader) waitForInt() tinygoerrors.ErrorCode {
	return waitForInt(pr.intPin, pr.logger)
}

// IsAvailableToRead checks if data is available on SPI
//
// Returns:
//
// True if data is available, otherwise false.
func (pr *SPIPacketReader) IsAvailableToRead() bool {
	if err := pr.waitForInt(); err != tinygoerrors.ErrorCodeNil {
		return false
	}
	return true
}

// readHeader reads the Packet header from the SPI bus.
//
// Returns:
//
// A PacketHeader or an error if reading the header fails.
func (pr *SPIPacketReader) readHeader() (PacketHeader, tinygoerrors.ErrorCode) {
	// Wait for INT pin to go low
	if err := pr.waitForInt(); err != tinygoerrors.ErrorCodeNil {
		return PacketHeader{}, err
	}

	// Check if the destination slice is nil
	packetBuffer := pr.packetBuffer.GetBuffer()
	if packetBuffer == nil {
		return PacketHeader{}, ErrorCodeBNO08XNilDestinationBuffer
	}

	// Check if start and end are within bounds
	if len(packetBuffer) < PacketHeaderLength {
		return PacketHeader{}, ErrorCodeBNO08XPacketBufferTooShortForPacketHeader
	}

	// Read the first 4 bytes from the SPI bus to get the Packet header.
	if err := pr.spiBus.Tx(
		nil,
		packetBuffer[:PacketHeaderLength],
	); err != nil {
		return PacketHeader{}, ErrorCodeBNO08XSPIFailedToReadPacketHeader
	}

	header, err := NewPacketHeaderFromBuffer(packetBuffer[:PacketHeaderLength])
	if err != tinygoerrors.ErrorCodeNil {
		return PacketHeader{}, err
	}

	// Debug log the header
	header.Log(false, pr.logger)
	return header, tinygoerrors.ErrorCodeNil
}

// ReadPacket reads a Packet from the SPI bus.
//
// Returns:
//
// A Packet or an error if reading the Packet fails.
func (pr *SPIPacketReader) ReadPacket() (Packet, tinygoerrors.ErrorCode) {
	// Read the Packet header
	header, errorCode := pr.readHeader()
	if errorCode != tinygoerrors.ErrorCodeNil {
		return Packet{}, errorCode
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
		if err := pr.spiBus.Tx(
			nil,
			dataBuffer,
		); err != nil {
			return Packet{}, ErrorCodeBNO08XSPIFailedToReadRequestedDataLength
		}
	}

	// Create a full Packet from the packet buffer
	packet, errorCode := NewPacket(dataBuffer, header)
	if errorCode != tinygoerrors.ErrorCodeNil {
		return Packet{}, errorCode
	}

	// Debug log the packet
	packet.Log(false, false, pr.logger)
	return packet, tinygoerrors.ErrorCodeNil
}

// newSPIPacketWriter creates a new SPIPacketWriter instance.
//
// Parameters:
//
// spiBus: The SPI bus to use for communication.
// intPin: The INT pin to monitor for data readiness.
// packetBuffer: The packet buffer to use for storing Packet data.
// logger: The logger to use for logging and debugging information.
//
// Returns:
//
// A pointer to a new SPIPacketWriter instance, or an error if the packetBuffer is nil.
func newSPIPacketWriter(
	spiBus *machine.SPI,
	intPin machine.Pin,
	packetBuffer PacketBuffer,
	logger tinygologger.Logger,
) (*SPIPacketWriter, tinygoerrors.ErrorCode) {
	// Check if the SPI bus is nil
	if spiBus == nil {
		return nil, ErrorCodeBNO08XNilSPIBus
	}

	// Check if the packetBuffer is provided
	if packetBuffer == nil {
		return nil, ErrorCodeBNO08XNilPacketBuffer
	}

	return &SPIPacketWriter{
		spiBus:       spiBus,
		intPin:       intPin,
		logger:       logger,
		packetBuffer: packetBuffer,
	}, tinygoerrors.ErrorCodeNil
}

// waitForInt waits for the INT pin to go low, indicating data is ready.
//
// Returns:
//
// An error if the wait times out.
func (pw *SPIPacketWriter) waitForInt() tinygoerrors.ErrorCode {
	return waitForInt(pw.intPin, pw.logger)
}

// SendPacket sends a packet over SPI, waiting for INT pin before sending.
//
// Parameters:
//
// channel: The channel to send the packet on.
// data: The data to send in the packet.
//
// Returns:
//
// The sequence number used and an error if any occurs.
func (pw *SPIPacketWriter) SendPacket(channel uint8, data []byte) (
	uint8,
	tinygoerrors.ErrorCode,
) {
	// Check if the data is nil
	if data == nil {
		return 0, ErrorCodeBNO08XNilPacketData
	}

	// Get channel sequence number
	sequenceNumber, errorCode := pw.packetBuffer.GetChannelSequenceNumber(channel)
	if errorCode != tinygoerrors.ErrorCodeNil {
		return 0, errorCode
	}

	// Initialize the packet from data
	packet, errorCode := NewPacketFromData(
		channel,
		sequenceNumber,
		data,
		pw.packetBuffer.GetBuffer()[:PacketHeaderLength], // Reuse header buffer
	)
	if errorCode != tinygoerrors.ErrorCodeNil {
		return 0, errorCode
	}

	// Debug log the packet
	packet.Log(true, true, pw.logger)

	// Wait for INT pin to go low before sending
	if errorCode := pw.waitForInt(); errorCode != tinygoerrors.ErrorCodeNil {
		return 0, errorCode
	}

	// Write to SPI
	if err := pw.spiBus.Tx(packet.Header.Buffer, nil); err != nil {
		return sequenceNumber, ErrorCodeBNO08XSPIFailedToWritePacketHeaderBuffer
	}
	if err := pw.spiBus.Tx(packet.Data, nil); err != nil {
		return sequenceNumber, ErrorCodeBNO08XSPIFailedToWritePacketPacketBuffer
	}

	// Update sequence number
	sequenceNumber, errorCode = pw.packetBuffer.IncrementChannelSequenceNumber(channel)
	if errorCode != tinygoerrors.ErrorCodeNil {
		return 0, errorCode
	}
	return sequenceNumber, tinygoerrors.ErrorCodeNil
}
