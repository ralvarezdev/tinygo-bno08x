//go:build tinygo && (rp2040 || rp2350)

package go_adafruit_bno08x

import (
	"fmt"

	"machine"
)

type (
	// I2C is the I2C implementation of the BNO08X sensor.
	I2C struct {
		BNO08X
		i2cBus  machine.I2C
		address uint16
	}

	// I2CPacketReader represents the Packet reader for the I2C interface.
	I2CPacketReader struct {
		i2cBus     machine.I2C
		dataBuffer DataBuffer
		debugger   Debugger
		address    uint16 // I2C address of the device
	}

	// I2CPacketWriter represents the Packet writer for the I2C interface.
	I2CPacketWriter struct {
		i2cBus     machine.I2C
		dataBuffer DataBuffer
		debugger   Debugger
		address    uint16 // I2C address of the device
	}
)

// NewI2C creates a new I2C instance for the BNO08X sensor.
//
// Parameters:
//
// i2cBus: The I2C bus to use for communication.
// sdaPin: The SDA pin for the I2C bus.
// sclPin: The SCL pin for the I2C bus.
// address: The I2C address of the BNO08X sensor.
// packetReader: The I2CPacketReader to use for reading Packets.
// packetWriter: The I2CPacketWriter to use for writing Packets.
// dataBuffer: The DataBuffer to use for storing Packet data.
// options: Optional configuration options for the BNO08X sensor.
//
// Returns:
//
// A pointer to a new I2C instance or an error if initialization fails.
func NewI2C(
	i2cBus machine.I2C,
	sdaPin machine.Pin,
	sclPin machine.Pin,
	address uint16,
	dataBuffer DataBuffer,
	options *Options,
) (*I2C, error) {
	// Configure the I2C bus
	i2cBus.Configure(
		machine.I2CConfig{
			SCL:       sclPin,
			SDA:       sdaPin,
			Frequency: I2CFrequency,
		},
	)

	// Get the debugger from options
	var debugger Debugger
	if options != nil {
		debugger = options.Debugger
	}

	// Initialize the packet reader
	packetReader, err := newI2CPacketReader(
		i2cBus,
		address,
		dataBuffer,
		debugger,
	)
	if err != nil {
		return nil, fmt.Errorf("failed to create I2CPacketReader: %w", err)
	}

	// Initialize the packet writer
	packetWriter, err := newI2CPacketWriter(
		i2cBus,
		address,
		dataBuffer,
		debugger,
	)
	if err != nil {
		return nil, fmt.Errorf("failed to create I2CPacketWriter: %w", err)
	}

	// Initialize the BNO08X sensor
	bno08x, err := NewBNO08X(packetReader, packetWriter, dataBuffer, options)
	if err != nil {
		return nil, fmt.Errorf("failed to initialize BNO08X: %w", err)
	}
	return &I2C{
		BNO08X:  *bno08x,
		i2cBus:  i2cBus,
		address: address,
	}, nil
}

// newI2CPacketWriter creates a new I2CPacketWriter instance.
//
// Parameters:
//
// i2cBus: The I2C bus to use for communication.
// address: The I2C address of the device to read from.
// dataBuffer: The data buffer to use for storing Packet data.
// debugger: The debugger to use for logging and debugging information.
//
// Returns:
//
// A pointer to a new I2CPacketWriter instance, or an error if the dataBuffer is nil.
func newI2CPacketWriter(
	i2cBus machine.I2C,
	address uint16,
	dataBuffer DataBuffer,
	debugger Debugger,
) (*I2CPacketWriter, error) {
	// Check if the dataBuffer is provided
	if dataBuffer == nil {
		return nil, ErrNilDataBuffer
	}

	return &I2CPacketWriter{
		i2cBus:     i2cBus,
		dataBuffer: dataBuffer,
		debugger:   debugger,
		address:    address,
	}, nil
}

// debug is a helper function to log debug messages if debugging is enabled.
//
// Parameters:
//
// args: The arguments to log as debug messages.
func (pw I2CPacketWriter) debug(args ...interface{}) {
	if pw.debugger != nil {
		pw.debugger.Debug(args...)
	}
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
func (pw I2CPacketWriter) SendPacket(channel uint8, data []byte) (
	uint8,
	error,
) {
	dataLength := len(data)
	writeLength := dataLength + 4

	// Ensure buffer is large enough
	dataBuffer := *pw.dataBuffer.GetData()
	if len(dataBuffer) < writeLength {
		dataBuffer = make([]byte, writeLength)
		pw.dataBuffer.SetData(&dataBuffer)
	}

	// Pack header: first two bytes are writeLength (little-endian)
	dataBuffer[0] = uint8(writeLength & 0xFF)
	dataBuffer[1] = uint8((writeLength >> 8) & 0xFF)
	dataBuffer[2] = channel
	sequenceNumber, err := pw.dataBuffer.GetSequenceNumber(channel)
	if err != nil {
		return 0, err
	}
	dataBuffer[3] = sequenceNumber

	// Copy data into buffer
	copy(dataBuffer[4:], data)

	// Create a new Packet from the data buffer
	dataBufferWriteLength := make([]byte, writeLength)
	copy(dataBufferWriteLength, dataBuffer[:writeLength])
	packet, err := NewPacket(&dataBufferWriteLength)
	if err != nil {
		return sequenceNumber, fmt.Errorf("failed to create Packet: %w", err)
	}
	pw.debug("Sending Packet:")
	pw.debug(packet)

	// Write to I2C
	if err = pw.i2cBus.Tx(pw.address, dataBufferWriteLength, nil); err != nil {
		return sequenceNumber, err
	}

	// Update sequence number
	sequenceNumber, err = pw.dataBuffer.IncrementChannelSequenceNumber(channel)
	if err != nil {
		return 0, err
	}
	return sequenceNumber, nil
}

// newI2CPacketReader creates a new I2CPacketReader instance.
//
// Parameters:
//
// i2cBus: The I2C bus to use for communication.
// debugger: The debugger to use for logging and debugging information.
// dataBuffer: The data buffer to use for storing Packet data.
// address: The I2C address of the device to read from.
//
// Returns:
//
// A pointer to a new I2CPacketReader instance.
func newI2CPacketReader(
	i2cBus machine.I2C,
	address uint16,
	dataBuffer DataBuffer,
	debugger Debugger,
) (*I2CPacketReader, error) {
	// Check if the dataBuffer is provided
	if dataBuffer == nil {
		return nil, ErrNilDataBuffer
	}

	return &I2CPacketReader{
		i2cBus:     i2cBus,
		debugger:   debugger,
		dataBuffer: dataBuffer,
		address:    address,
	}, nil
}

// debug is a helper function to log debug messages if debugging is enabled.
//
// Parameters:
//
// args: The arguments to log as debug messages.
func (pr I2CPacketReader) debug(args ...interface{}) {
	if pr.debugger != nil {
		pr.debugger.Debug(args)
	}
}

// readHeader reads the Packet header from the I2C bus.
//
// Returns:
//
// A pointer to a PacketHeader or an error if reading the header fails.
func (pr I2CPacketReader) readHeader() (*PacketHeader, error) {
	// Ensure the data buffer is at least 4 bytes long to read the header
	bufPtr := pr.dataBuffer.GetData()
	if len(*buf) < 4 {
		newBuf := make([]byte, 4)
		pr.dataBuffer.SetData(&newBuf)
		bufPtr = &newBuf
	}

	// Read the first 4 bytes from the I2C bus to get the Packet header.
	if err := pr.i2cBus.Tx(pr.address, nil, pr.dataBuffer[:4]); err != nil {
		return nil, err
	}
	header, err := NewPacketHeader(pr.dataBuffer.GetData())
	if err != nil {
		return nil, err
	}
	pr.debug(header)
	return header, nil
}

func (pr I2CPacketReader) ReadPacket() (*Packet, error) {
	// Read the Packet header first
	header, err := pr.readHeader()
	if err != nil {
		return nil, err
	}

	packetByteCount := header.PacketByteCount
	channelNumber := header.ChannelNumber
	sequenceNumber := header.SequenceNumber

	// Set sequence number in data buffer
	if err = pr.dataBuffer.SetSequenceNumber(
		channelNumber,
		sequenceNumber,
	); err != nil {
		return nil, fmt.Errorf("failed to set sequence number: %w", err)
	}
	if packetByteCount == 0 {
		pr.debug("SKIPPING NO PACKETS AVAILABLE IN i2c.readPacket")
		return nil, ErrNoPacketAvailable
	}

	// packetByteCount includes 4 header bytes
	payloadLen := packetByteCount - 4
	pr.debug(
		"channel",
		channelNumber,
		"has",
		payloadLen,
		"bytes available to read",
	)

	// Read the remaining bytes of the Packet
	if err = pr.read(packpayloadLen); err != nil {
		return nil, fmt.Errorf("failed to read Packet data: %w", err)
	}

	// Create a full Packet from the data buffer
	newPacket, err := NewPacket(pr.dataBuffer.GetData())
	if err != nil {
		return nil, fmt.Errorf("failed to create Packet from bytes: %w", err)
	}
	pr.debug(*newPacket)

	// Update the sequence number in the data buffer
	if err = pr.dataBuffer.UpdateSequenceNumber(newPacket); err != nil {
		return nil, fmt.Errorf("failed to update sequence number: %w", err)
	}
	return newPacket, nil
}

// read reads a specified number of bytes from the I2C bus.
//
// Parameters:
//
// requestedReadLength: The number of bytes to read from the I2C bus.
//
// Returns:
//
// An error if reading from the I2C bus fails, otherwise nil.
func (pr I2CPacketReader) read(requestedReadLength int) error {
	pr.debug("trying to read", requestedReadLength, "bytes")
	totalReadLength := requestedReadLength + 4
	if totalReadLength > DataBufferSize {
		newDataBuffer := make([]byte, DataBufferSize)
		pr.dataBuffer.SetData(&newDataBuffer)
		pr.debug(
			fmt.Sprintf(
				"!!!!!!!!!!!! ALLOCATION: increased dataBuffer to %d !!!!!!!!!!!!!",
				totalReadLength,
			),
		)
	}
	dataBuffer := pr.dataBuffer.GetData()

	// Preserve first 4 header bytes already read; read payload into slice after header.
	if requestedReadLength > 0 {
		if err := pr.i2cBus.Tx(
			pr.address,
			nil,
			(*dataBuffer)[4:totalReadLength],
		); err != nil {
			return err
		}
	}
	return nil
}

// IsDataReady checks if there is data ready to be read from the I2C bus.
//
// Returns:
//
// True if data is ready, false otherwise. It also checks for errors in the header.
func (pr I2CPacketReader) IsDataReady() bool {
	header, err := pr.readHeader()
	if err != nil {
		pr.debug("error reading header:", err)
		return false
	}
	if header.ChannelNumber > 5 {
		pr.debug("channel number out of range:", header.ChannelNumber)
	}
	if header.PacketByteCount == 0x7FFF {
		fmt.Println("Byte count is 0x7FFF/0xFFFF; Error?")
		if header.SequenceNumber == 0xFF {
			fmt.Println("Sequence number is 0xFF; Error?")
		}
		return false
	}
	return header.DataLength > 0
}
