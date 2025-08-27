//go:build tinygo && (rp2040 || rp2350)

package go_bno08x

import (
	"fmt"
	"time"

	"machine"
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
		dataBuffer DataBuffer
		debugger   Debugger
		ultraDebug bool
	}

	// UARTPacketWriter is the packet writer for UART interface
	UARTPacketWriter struct {
		uartBus    *machine.UART
		dataBuffer DataBuffer
		debugger   Debugger
		ultraDebug bool
	}

	// UARTOptions struct for configuring the BNO08X over UART.
	UARTOptions struct {
		Options    *Options
		UltraDebug bool
	}
)

// NewUARTOptions creates a new UARTOptions instance with default values.
//
// Parameters:
//
// debugger: The debugger to use for logging and debugging information (optional).
// ultraDebug: Flag to enable ultra debug mode (optional).
//
// Returns:
//
// A pointer to a new UARTOptions instance.
func NewUARTOptions(
	debugger Debugger,
	ultraDebug bool,
) *UARTOptions {
	return &UARTOptions{
		Options:    NewOptions(debugger),
		UltraDebug: ultraDebug,
	}
}

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
// dataBuffer: The data buffer to use for storing Packet data.
//
//	afterResetFn: An optional function to be called after a reset.
//
// options: The UARTOptions for configuring the BNO08X (optional).
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
	dataBuffer DataBuffer,
	afterResetFn func(b *BNO08X) error,
	options *UARTOptions,
) (*UART, error) {
	// Check if the UART bus is nil
	if uartBus == nil {
		return nil, ErrNilUARTBus
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
		return nil, fmt.Errorf("failed to configure uart: %w", err)
	}

	// If options are nil, initialize with default values
	if options == nil {
		options = NewUARTOptions(nil, false)
	}

	// Get the debugger from options
	debugger := options.Options.Debugger

	// Create packet reader and writer
	packetReader, err := newUARTPacketReader(
		uartBus,
		dataBuffer,
		debugger,
		options.UltraDebug,
	)
	if err != nil {
		return nil, fmt.Errorf("failed to create uart packet reader: %w", err)
	}

	packetWriter, err := newUARTPacketWriter(
		uartBus,
		dataBuffer,
		debugger,
		options.UltraDebug,
	)
	if err != nil {
		return nil, fmt.Errorf("failed to create uart packet writer: %w", err)
	}

	// Initialize BNO08X
	bno08x, err := NewBNO08X(
		resetPin,
		packetReader,
		packetWriter,
		dataBuffer,
		afterResetFn,
		options.Options,
	)
	if err != nil {
		return nil, fmt.Errorf("failed to initialize bno08x: %w", err)
	}

	return &UART{
		BNO08X:   bno08x,
		uartBus:  uartBus,
		ps1Pin:   ps1Pin,
		ps0Pin:   ps0Pin,
		resetPin: resetPin,
	}, nil
}

// GetBNO08XService returns the BNO08X service.
//
// Returns:
//
// The BNO08X service instance.
func (uart *UART) GetBNO08XService() BNO08XService {
	return uart.BNO08X
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
// debugger: The debugger to use for logging and debugging information.
// dataBuffer: The data buffer to use for storing Packet data.
// ultraDebug: Flag to enable ultra debug mode (optional).
//
// Returns:
//
// A pointer to a new UARTPacketReader instance.
func newUARTPacketReader(
	uartBus *machine.UART,
	dataBuffer DataBuffer,
	debugger Debugger,
	ultraDebug bool,
) (*UARTPacketReader, error) {
	// Check if the UART bus is nil
	if uartBus == nil {
		return nil, ErrNilUARTBus
	}

	// Check if the dataBuffer is provided
	if dataBuffer == nil {
		return nil, ErrNilDataBuffer
	}

	return &UARTPacketReader{
		uartBus:    uartBus,
		debugger:   debugger,
		dataBuffer: dataBuffer,
		ultraDebug: ultraDebug,
	}, nil
}

// IsDataReady checks if data is available on UART
//
// Returns:
//
// True if data is available, otherwise false.
func (pr *UARTPacketReader) IsDataReady() bool {
	return pr.uartBus.Buffered() >= PacketHeaderLength
}

// readByte blocks until a byte is read (simple poll).
//
// Returns:
//
// A byte read from UART and an error if any.
func (pr *UARTPacketReader) readByte() (byte, error) {
	startTime := time.Now()
	for time.Since(startTime) < UARTByteTimeout {
		if pr.uartBus.Buffered() > 0 {
			b, err := pr.uartBus.ReadByte()
			if pr.debugger != nil && pr.ultraDebug {
				pr.debugger.Debug(fmt.Sprintf("Received byte: 0x%02X", b))
			}
			return b, err
		}
		time.Sleep(1 * time.Millisecond)
	}
	return 0, ErrUARTTimeout
}

// readInto reads bytes into the destination buffer handling escape sequences.
//
// Parameters:
//
// dst: The destination byte slice to read into.
// start: The starting index in the destination slice.
// end: The ending index in the destination slice (optional).
//
// Returns:
//
// An error if any occurs during reading.
func (pr *UARTPacketReader) readInto(dst *[]byte, start int, end *int) error {
	// Check if the dst is nil
	if dst == nil {
		return ErrNilDestinationBuffer
	}

	// Determine end index
	if end == nil {
		end = new(int)
		*end = len(*dst)
	}

	for i := start; i < *end; i++ {
		b, err := pr.readByte()
		if err != nil {
			return err
		}
		if b == UARTControlEscape {
			nb, err := pr.readByte()
			if err != nil {
				return err
			}
			b = nb ^ 0x20
		}
		(*dst)[i] = b
	}
	return nil
}

// readHeader reads the UART packet header.
//
// Returns:
//
// An error if any occurs during reading.
func (pr *UARTPacketReader) readHeader() error {
	// Find first initial start byte
	for {
		b, err := pr.readByte()
		if err != nil {
			return err
		}
		if b == UARTStartAndEndByte {
			break
		}
	}

	// Read protocol ID sequence
	data, err := pr.readByte()
	if err != nil {
		return err
	}
	if data == UARTStartAndEndByte {
		// Consume next (real protocol byte)
		data, err = pr.readByte()
		if err != nil {
			return err
		}
	}
	if data != UARTSHTPByte {
		return ErrUnhandledUARTControlSHTPProtocol
	}
	end := PacketHeaderLength

	return pr.readInto(pr.dataBuffer.GetData(), 0, &end)
}

// ReadPacket reads a packet from UART
//
// Returns:
//
// A Packet object and an error if any occurs.
func (pr *UARTPacketReader) ReadPacket() (*Packet, error) {
	// Read packet header
	if err := pr.readHeader(); err != nil {
		return nil, err
	}

	// Parse header
	header, err := NewPacketHeaderFromBuffer(pr.dataBuffer.GetData())
	if err != nil {
		return nil, err
	}
	if header.PacketByteCount == 0 {
		return nil, ErrNoPacketAvailable
	}
	channelNumber := header.ChannelNumber

	// Debug log the header
	if pr.debugger != nil && pr.ultraDebug {
		headerStrPtr := header.String(false)
		if headerStrPtr != nil {
			pr.debugger.Debug(*headerStrPtr)
		} else {
			pr.debugger.Debug(ErrNilPacketHeaderString.Error())
		}

		// Log available bytes
		pr.debugger.Debug(
			fmt.Sprintf(
				"Channel %d has %d bytes available",
				channelNumber,
				header.PacketByteCount-PacketHeaderLength,
			),
		)
	}

	// Read remaining (payload) bytes
	end := int(header.PacketByteCount)
	dataBuffer := pr.dataBuffer.GetData()
	if err = pr.readInto(
		dataBuffer,
		PacketHeaderLength,
		&end,
	); err != nil {
		return nil, err
	}

	// Expect trailing 0x7E
	endByte, err := pr.readByte()
	if err != nil {
		return nil, err
	}
	if endByte != UARTStartAndEndByte {
		return nil, ErrUARTEndMissing
	}

	// Construct packet data
	packetData := make([]byte, header.DataLength)
	copy(packetData, (*dataBuffer)[PacketHeaderLength:header.PacketByteCount])

	// Initialize packet
	packet, err := NewPacket(&packetData, header)
	if err != nil {
		return nil, fmt.Errorf("failed to create packet from bytes: %w", err)
	}

	// Debug log the packet
	if pr.debugger != nil {
		packetStrPtr := packet.String(false)
		if packetStrPtr != nil {
			pr.debugger.Debug(*packetStrPtr)
		} else {
			pr.debugger.Debug(ErrNilPacketString.Error())
		}
	}

	// Update sequence number
	pr.dataBuffer.UpdateSequenceNumber(packet)
	return packet, nil
}

// newUARTPacketWriter creates a new UARTPacketWriter instance.
//
// Parameters:
//
// uartBus: The UART bus to use for communication.
// dataBuffer: The data buffer to use for storing Packet data.
// debugger: The debugger to use for logging and debugging information.
// ultraDebug: Flag to enable ultra debug mode (optional).
//
// Returns:
//
// A pointer to a new UARTPacketWriter instance, or an error if the dataBuffer is nil.
func newUARTPacketWriter(
	uartBus *machine.UART,
	dataBuffer DataBuffer,
	debugger Debugger,
	ultraDebug bool,
) (*UARTPacketWriter, error) {
	// Check if the UART bus is nil
	if uartBus == nil {
		return nil, ErrNilUARTBus
	}

	// Check if the dataBuffer is provided
	if dataBuffer == nil {
		return nil, ErrNilDataBuffer
	}

	return &UARTPacketWriter{
		uartBus:    uartBus,
		debugger:   debugger,
		dataBuffer: dataBuffer,
		ultraDebug: ultraDebug,
	}, nil
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
func (pw *UARTPacketWriter) SendPacket(channel uint8, data *[]byte) (
	uint8,
	error,
) {
	// Get channel sequence number
	sequenceNumber, err := pw.dataBuffer.GetSequenceNumber(channel)
	if err != nil {
		return 0, err
	}

	// Initialize the packet from data
	packet, err := NewPacketFromData(
		channel,
		sequenceNumber,
		data,
	)
	if err != nil {
		return 0, fmt.Errorf("failed to create packet: %w", err)
	}

	// Debug log the packet
	if pw.debugger != nil {
		packetStrPtr := packet.String(true)
		if packetStrPtr != nil {
			pw.debugger.Debug(*packetStrPtr)
		} else {
			pw.debugger.Debug(ErrNilPacketString.Error())
		}
	}

	// Get the packet buffer
	packetBufferPtr := packet.Buffer()
	if packetBufferPtr == nil {
		return 0, ErrNilPacketBuffer
	}

	// Send start byte
	pw.uartBus.WriteByte(UARTStartAndEndByte)
	time.Sleep(1 * time.Millisecond)

	// Send SHTP protocol byte
	pw.uartBus.WriteByte(UARTSHTPByte)
	time.Sleep(1 * time.Millisecond)

	// Send packet with escape sequences
	for _, b := range *packetBufferPtr {
		pw.uartBus.WriteByte(b)
		time.Sleep(1 * time.Millisecond)
	}

	// Send start byte
	pw.uartBus.WriteByte(UARTStartAndEndByte)
	time.Sleep(1 * time.Millisecond)

	// Update sequence number
	sequenceNumber, err = pw.dataBuffer.IncrementChannelSequenceNumber(channel)
	if err != nil {
		return 0, err
	}
	return sequenceNumber, nil
}
