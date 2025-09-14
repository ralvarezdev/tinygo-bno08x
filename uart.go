//go:build tinygo && (rp2040 || rp2350)

package tinygo_bno08x

import (
	"time"

	"machine"

	tinygotypes "github.com/ralvarezdev/tinygo-types"
	tinygologger "github.com/ralvarezdev/tinygo-logger"
)

type (
	// UART is the UART implementation of the BNO08X sensor
	UART struct {
		*BNO08X
		uartBus  *machine.UART
		ps0Pin   machine.Pin
		ps1Pin   machine.Pin
		resetPin machine.Pin
	}

	// UARTPacketReader is the packet reader for UART interface
	UARTPacketReader struct {
		uartBus    *machine.UART
		packetBuffer PacketBuffer
		logger   tinygologger.Logger
		ultraDebug bool
	}

	// UARTPacketWriter is the packet writer for UART interface
	UARTPacketWriter struct {
		uartBus    *machine.UART
		packetBuffer PacketBuffer
		logger   tinygologger.Logger
		ultraDebug bool
	}
)

// NewUART creates a new UART instance for the BNO08X sensor
//
// Parameters:
//
// uartBus: The UART bus to use for communication.
// txPin: The TX pin for UART communication.
// rxPin: The RX pin for UART communication.
// ps0Pin: The PS0 pin to set the sensor to UART mode.
// ps1Pin: The PS1 pin to set the sensor to UART mode.
// resetPin: The pin used to reset the BNO08X sensor.
// packetBuffer: The packet buffer to use for storing Packet data.
// afterResetFn: An optional function to be called after a reset.
// logger: The logger to use for logging and debugging information (optional).
// ultraDebug: Flag to enable ultra debug mode (optional).
//
// Returns:
//
// A pointer to a new UART instance and an error if any occurs.
func NewUART(
	uartBus *machine.UART,
	txPin machine.Pin,
	rxPin machine.Pin,
	ps0Pin machine.Pin,
	ps1Pin machine.Pin,
	resetPin machine.Pin,
	packetBuffer PacketBuffer,
	afterResetFn func(b *BNO08X) tinygotypes.ErrorCode,
	logger tinygologger.Logger,
	ultraDebug bool,
) (*UART, tinygotypes.ErrorCode) {
	// Check if the UART bus is nil
	if uartBus == nil {
		return nil, ErrorCodeBNO08XNilUARTBus
	}

	// Set PS0 pin to output and low
	ps0Pin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	ps0Pin.Low()

	// Set PS1 pin to output and high
	ps1Pin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	ps1Pin.High()

	// Configure UART
	if err := uartBus.Configure(
		machine.UARTConfig{
			BaudRate: UARTBaudRate,
			TX:       txPin,
			RX:       rxPin,
		},
	); err != nil {
		return nil, ErrorCodeBNO08XFailedToConfigureUART
	}

	// Set UART format (8N1)
	if err := uartBus.SetFormat(UARTDataBits, UARTStopBits, UARTParity); err != nil {
		return nil, ErrorCodeBNO08XFailedToSetUARTFormat
	}

	// Create packet reader and writer
	packetReader, err := newUARTPacketReader(
		uartBus,
		packetBuffer,
		logger,
		ultraDebug,
	)
	if err != tinygotypes.ErrorCodeNil {
		return nil, ErrorCodeBNO08XFailedToCreatePacketReader
	}

	packetWriter, err := newUARTPacketWriter(
		uartBus,
		packetBuffer,
		logger,
		ultraDebug,
	)
	if err != tinygotypes.ErrorCodeNil {
		return nil, ErrorCodeBNO08XFailedToCreatePacketWriter
	}

	// Initialize BNO08X
	bno08x, err := NewBNO08X(
		resetPin,
		packetReader,
		packetWriter,
		packetBuffer,
		UARTMode,
		afterResetFn,
		logger,
	)
	if err != tinygotypes.ErrorCodeNil {
		return nil, err
	}

	return &UART{
		BNO08X:   bno08x,
		uartBus:  uartBus,
		ps1Pin:   ps1Pin,
		ps0Pin:   ps0Pin,
		resetPin: resetPin,
	}, tinygotypes.ErrorCodeNil
}

// GetBNO08X returns the BNO08X instance.
//
// Returns:
//
// The BNO08X instance.
func (uart *UART) GetBNO08X() *BNO08X {
	return uart.BNO08X
}

// newUARTPacketReader creates a new UARTPacketReader instance.
//
// Parameters:
//
// uartBus: The UART bus to use for communication.
// logger: The logger to use for logging and debugging information.
// packetBuffer: The packet buffer to use for storing Packet data.
// ultraDebug: Flag to enable ultra debug mode (optional).
//
// Returns:
//
// A pointer to a new UARTPacketReader instance.
func newUARTPacketReader(
	uartBus *machine.UART,
	packetBuffer PacketBuffer,
	logger tinygologger.Logger,
	ultraDebug bool,
) (*UARTPacketReader, tinygotypes.ErrorCode) {
	// Check if the UART bus is nil
	if uartBus == nil {
		return nil, ErrorCodeBNO08XNilUARTBus
	}

	// Check if the packetBuffer is provided
	if packetBuffer == nil {
		return nil, ErrorCodeBNO08XNilPacketBuffer
	}

	return &UARTPacketReader{
		uartBus:    uartBus,
		logger:   logger,
		packetBuffer: packetBuffer,
		ultraDebug: ultraDebug,
	}, tinygotypes.ErrorCodeNil
}

var (
	// receivedBytePrefix is the prefix for received bytes in debug logs
	receivedBytePrefix = []byte("Received byte:")

	// sentBytePrefix is the prefix for sent bytes in debug logs
	sentBytePrefix = []byte("Sent byte:")
)

// IsAvailableToRead checks if data is available on UART
//
// Returns:
//
// True if data is available, otherwise false.
func (pr *UARTPacketReader) IsAvailableToRead() bool {
	return pr.uartBus.Buffered() >= 4
}

// readByte blocks until a byte is read (simple poll).
//
// Returns:
//
// A byte read from UART and an error if any.
func (pr *UARTPacketReader) readByte() (byte, tinygotypes.ErrorCode) {
	startTime := time.Now()
	for time.Since(startTime) < UARTByteTimeout {
		if pr.uartBus.Buffered() > 0 {
			b, err := pr.uartBus.ReadByte()
			if err != nil {
				return b, ErrorCodeBNO08XUARTFailedToReadByte
			}
			if pr.logger != nil && pr.ultraDebug {
				pr.logger.AddMessageWithUint8(receivedBytePrefix, b, true, true, true)
				pr.logger.Debug()
			}
			return b, tinygotypes.ErrorCodeNil
		}
	}
	return 0, ErrorCodeBNO08XUARTByteTimeout
}

// readInto reads bytes into the buffer buffer handling escape sequences.
//
// Parameters:
//
// buffer: The buffer byte slice to read into.
// start: The starting index in the buffer slice.
// end: The ending index in the buffer slice.
//
// Returns:
//
// An error if any occurs during reading.
func (pr *UARTPacketReader) readInto(buffer []byte, start int, end int) tinygotypes.ErrorCode {
	// Check if the buffer slice is nil
	if buffer == nil {
		return ErrorCodeBNO08XNilDestinationBuffer
	}

	// Check if start and end are within bounds
	if start < 0 || end > len(buffer) || start >= end {
		return ErrorCodeBNO08XInvalidStartOrEndIndex
	}

	// Read bytes into the buffer slice
	for i := start; i < end; i++ {
		b, err := pr.readByte()
		if err != tinygotypes.ErrorCodeNil {
			return err
		}
		if b == UARTControlEscape {
			nb, err := pr.readByte()
			if err != tinygotypes.ErrorCodeNil {
				return err
			}
			b = nb ^ 0x20
		}
		buffer[i] = b
	}
	return tinygotypes.ErrorCodeNil
}

// readHeader reads the UART packet header.
//
// Returns:
//
// An error if any occurs during reading.
func (pr *UARTPacketReader) readHeader() tinygotypes.ErrorCode {
	// Find first initial start byte
	for {
		b, err := pr.readByte()
		if err != tinygotypes.ErrorCodeNil {
			return err
		}
		if b == UARTStartAndEndByte {
			break
		}
	}

	// Read protocol ID sequence
	data, err := pr.readByte()
	if err != tinygotypes.ErrorCodeNil {
		return err
	}
	if data == UARTStartAndEndByte {
		// Consume next (real protocol byte)
		data, err = pr.readByte()
		if err != tinygotypes.ErrorCodeNil {
			return err
		}
	}
	if data != UARTSHTPByte {
		return ErrorCodeBNO08XUnhandledUARTControlSHTPProtocol
	}
	return pr.readInto(pr.packetBuffer.GetBuffer(), 0, PacketHeaderLength)
}

// ReadPacket reads a packet from UART
//
// Returns:
//
// A Packet object and an error if any occurs.
func (pr *UARTPacketReader) ReadPacket() (Packet, tinygotypes.ErrorCode) {
	// Read packet header
	if err := pr.readHeader(); err != tinygotypes.ErrorCodeNil {
		return Packet{}, err
	}

	// Parse header
	packetBuffer := pr.packetBuffer.GetBuffer()
	header, err := NewPacketHeaderFromBuffer(packetBuffer[:PacketHeaderLength])
	if err != tinygotypes.ErrorCodeNil {
		return Packet{}, err
	}
	if header.PacketByteCount == 0 {
		return Packet{}, ErrorCodeBNO08XNoPacketAvailable
	}
	channelNumber := header.ChannelNumber

	// Check if the channel number is valid
	if channelNumber > MaxChannelNumber {
		return Packet{}, ErrorCodeBNO08XInvalidChannelNumber
	}

	// Check the data length for the packet
	if header.DataLength > MaxDataLength {
		return Packet{}, ErrorCodeBNO08XInvalidReportDataLength
	}

	// Log the header
	header.Log(false, pr.logger)

	// Read remaining (payload) bytes
	if err = pr.readInto(
		packetBuffer[PacketHeaderLength:header.PacketByteCount],
		0,
		int(header.DataLength),
	); err != tinygotypes.ErrorCodeNil {
		return Packet{}, err
	}

	// Expect trailing 0x7E
	endByte, err := pr.readByte()
	if err != tinygotypes.ErrorCodeNil {
		return Packet{}, err
	}
	if endByte != UARTStartAndEndByte {
		return Packet{}, ErrorCodeBNO08XUARTEndMissing
	}

	// Initialize packet
	packet, err := NewPacket(packetBuffer[PacketHeaderLength:header.PacketByteCount], header)
	if err != tinygotypes.ErrorCodeNil {
		return Packet{}, err
	}

	// Log the packet
	packet.Log(false, false, pr.logger)
	return packet, tinygotypes.ErrorCodeNil
}

// newUARTPacketWriter creates a new UARTPacketWriter instance.
//
// Parameters:
//
// uartBus: The UART bus to use for communication.
// packetBuffer: The packet buffer to use for storing Packet data.
// logger: The logger to use for logging and debugging information.
// ultraDebug: Flag to enable ultra debug mode (optional).
//
// Returns:
//
// A pointer to a new UARTPacketWriter instance, or an error if the packetBuffer is nil.
func newUARTPacketWriter(
	uartBus *machine.UART,
	packetBuffer PacketBuffer,
	logger tinygologger.Logger,
	ultraDebug bool,
) (*UARTPacketWriter, tinygotypes.ErrorCode) {
	// Check if the UART bus is nil
	if uartBus == nil {
		return nil, ErrorCodeBNO08XNilUARTBus
	}

	// Check if the packetBuffer is provided
	if packetBuffer == nil {
		return nil, ErrorCodeBNO08XNilPacketBuffer
	}

	return &UARTPacketWriter{
		uartBus:    uartBus,
		logger:   logger,
		packetBuffer: packetBuffer,
		ultraDebug: ultraDebug,
	}, tinygotypes.ErrorCodeNil
}

// writeByte writes a byte to UART.
//
// Parameters:
//
// b: The byte to write.
// isData: Flag indicating if the byte is data (true) or control (false).
func (pw *UARTPacketWriter) writeByte(b byte, isData bool) {
	// Escape byte if needed
	if isData && (b == UARTStartAndEndByte || b == UARTControlEscape) {
		pw.uartBus.WriteByte(UARTControlEscape)
		time.Sleep(UARTByteDelay)
		b ^= 0x20
	}

	// Write byte to UART
	pw.uartBus.WriteByte(b)
	if pw.logger != nil && pw.ultraDebug {
		pw.logger.AddMessageWithUint8(sentBytePrefix, b, true, true, true)
		pw.logger.Debug()
	}
	time.Sleep(UARTByteDelay)
}

// SendPacket sends a packet over UART
//
// Parameters:
//
// channel: The channel to send the packet on.
// data: The data to send in the packet.
//
// Returns:
//
// The sequence number used and an error if any occurs.
func (pw *UARTPacketWriter) SendPacket(channel uint8, data []byte) (
	uint8,
	tinygotypes.ErrorCode,
) {
	// Check if the data is nil
	if data == nil {
		return 0, ErrorCodeBNO08XNilPacketData
	}

	// Get channel sequence number
	sequenceNumber, err := pw.packetBuffer.GetChannelSequenceNumber(channel)
	if err != tinygotypes.ErrorCodeNil {
		return 0, err
	}

	// Initialize the packet from data
	packet, err := NewPacketFromData(
		channel,
		sequenceNumber,
		data,
		pw.packetBuffer.GetBuffer()[:PacketHeaderLength], // Reuse header buffer
	)
	if err != tinygotypes.ErrorCodeNil {
		return 0, err
	}

	// Log the packet
	packet.Log(true, true, pw.logger)

	// Send start byte
	pw.writeByte(UARTStartAndEndByte, false)

	// Send SHTP protocol byte
	pw.writeByte(UARTSHTPByte, false)

	// Send the packet header
	for _, b := range packet.Header.Buffer {
		pw.writeByte(b, true)
	}

	// Send the packet data
	for _, b := range packet.Data {
		pw.writeByte(b, true)
	}

	// Send start byte
	pw.writeByte(UARTStartAndEndByte, false)

	// Update sequence number
	sequenceNumber, err = pw.packetBuffer.IncrementChannelSequenceNumber(channel)
	if err != tinygotypes.ErrorCodeNil {
		return 0, err
	}
	return sequenceNumber, tinygotypes.ErrorCodeNil
}
