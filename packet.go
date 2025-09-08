//go:build tinygo && (rp2040 || rp2350)

package tinygo_bno08x

import (
	"encoding/binary"
	"fmt"
	"strings"
)

type (
	// PacketHeader represents the header of a BNO08x Packet
	PacketHeader struct {
		ChannelNumber   uint8
		SequenceNumber  uint8
		DataLength      int
		PacketByteCount int
	}

	// Packet represents a BNO08x Packet
	Packet struct {
		Header *PacketHeader
		Data   []byte
	}
)

// NewPacketHeader creates a PacketHeader.
//
// Parameters:
//
// packetByteCount: The total byte count of the Packet.
// channelNumber: The channel number of the Packet.
// sequenceNumber: The sequence number of the Packet.
//
// Returns:
//
// A PacketHeader object.
func NewPacketHeader(
	packetByteCount uint16,
	channelNumber uint8,
	sequenceNumber uint8,
) *PacketHeader {
	dataLength := int(packetByteCount) - PacketHeaderLength
	if dataLength < 0 {
		dataLength = 0
	}

	return &PacketHeader{
		ChannelNumber:   channelNumber,
		SequenceNumber:  sequenceNumber,
		DataLength:      dataLength,
		PacketByteCount: int(packetByteCount),
	}
}

// NewPacketHeaderFromBuffer creates a PacketHeader from a given buffer.
//
// Parameters:
//
//	packetBytes: A pointer to a byte slice containing the Packet data.
//
// Returns:
//
//	A PacketHeader object or an error if the buffer is too short.
func NewPacketHeaderFromBuffer(packetBytes *[]byte) (*PacketHeader, error) {
	// Check if the provided packetBytes is nil
	if packetBytes == nil {
		return nil, ErrNilPacketBytes
	}

	// Ensure the buffer is at least PacketHeaderLength bytes long to read the header
	if len(*packetBytes) < 4 {
		return nil, ErrBufferTooShortForHeader
	}

	packetByteCount := binary.LittleEndian.Uint16((*packetBytes)[0:2])
	packetByteCount &= 0x7FFF
	channelNumber := (*packetBytes)[2]
	sequenceNumber := (*packetBytes)[3]

	return NewPacketHeader(
		packetByteCount,
		channelNumber,
		sequenceNumber,
	), nil
}

// NewPacketHeaderFromData creates a PacketHeader from the provided data.
//
// Parameters:
//
// channelNumber: The channel number of the Packet.
// sequenceNumber: The sequence number of the Packet.
// data: A pointer to a byte slice containing the Packet data.
func NewPacketHeaderFromData(
	channelNumber uint8,
	sequenceNumber uint8,
	data *[]byte,
) (*PacketHeader, error) {
	// Check if data is nil
	if data == nil {
		return nil, ErrNilDataBuffer
	}

	// Calculate packet byte count
	packetByteCount := len(*data) + PacketHeaderLength

	return NewPacketHeader(
		uint16(packetByteCount),
		channelNumber,
		sequenceNumber,
	), nil
}

// Buffer returns the byte representation of the PacketHeader.
//
// Returns:
//
// A pointer to a byte slice containing the PacketHeader bytes.
func (h *PacketHeader) Buffer() *[]byte {
	// Initialize header buffer
	buffer := make([]byte, PacketHeaderLength)

	// First two bytes are writeLength (little-endian)
	buffer[0] = uint8(h.PacketByteCount & 0xFF)
	buffer[1] = uint8((h.PacketByteCount >> 8) & 0x7F)
	buffer[2] = h.ChannelNumber
	buffer[3] = h.SequenceNumber

	return &buffer
}

// IsError checks if the provided PacketHeader indicates an error condition.
//
// Parameters:
//
//	header: The PacketHeader to check.
//
// Returns:
//
//	True if the header indicates an error, otherwise false.
func (h *PacketHeader) IsError() bool {
	// Check if the channel number is greater than 5
	if h.ChannelNumber > 5 {
		return true
	}
	// Check if the Packet byte count and sequence number indicate an error
	if h.PacketByteCount == 0xFFFF && h.SequenceNumber == 0xFF {
		return true
	}
	return false
}

// String returns a string representation of the PacketHeader for debugging purposes.
//
// Parameters:
//
// isBeingSent: A boolean indicating if the PacketHeader is being sent (true) or received (false).
//
// Returns:
//
// A string containing the PacketHeader details.
func (ph *PacketHeader) String(isBeingSent bool) *string {
	if ph == nil {
		return nil
	}
	var builder strings.Builder
	if isBeingSent {
		builder.WriteString("********** SENDING PACKET HEADER *************")
	} else {
		builder.WriteString("********** RECEIVED PACKET HEADER *************")
	}
	builder.WriteString(fmt.Sprintf("\n\t Data Length: %d", ph.DataLength))
	if int(ph.ChannelNumber) < len(Channels) {
		builder.WriteString(
			fmt.Sprintf(
				"\n\t Channel: %s (%d)",
				Channels[ph.ChannelNumber],
				ph.ChannelNumber,
			),
		)
	} else {
		builder.WriteString(
			fmt.Sprintf(
				"\n\t Channel: UNKNOWN (%d)",
				ph.ChannelNumber,
			),
		)
	}
	builder.WriteString(
		fmt.Sprintf(
			"\n\t Sequence number: %d",
			ph.SequenceNumber,
		),
	)
	builder.WriteString("\n\t *******************************")
	str := builder.String()
	return &str
}

// NewPacket creates a new Packet from the provided data and header.
//
// Parameters:
//
//	data: A pointer to a byte slice containing the Packet data.
//	header: A pointer to the PacketHeader.
//
// Returns:
//
// A Packet object or an error if the data or header is nil.
func NewPacket(data *[]byte, h *PacketHeader) (*Packet, error) {
	// Check if the provided data is nil
	if data == nil {
		return nil, ErrNilPacketData
	}

	// Check if the provided header is nil
	if h == nil {
		return nil, ErrNilPacketHeader
	}

	return &Packet{
		Header: h,
		Data:   *data,
	}, nil
}

// NewPacketFromBuffer creates a new Packet from the provided buffer.
//
// Parameters:
//
//	packetBytes: A pointer to a byte slice containing the Packet data.
//
// Returns:
//
//	A Packet object or an error if the Packet header could not be created.
func NewPacketFromBuffer(packetBytes *[]byte) (*Packet, error) {
	// Check if the provided packetBytes is nil
	if packetBytes == nil {
		return nil, ErrNilPacketBytes
	}

	// Create a new PacketHeader from the Packet bytes
	h, err := NewPacketHeaderFromBuffer(packetBytes)
	if err != nil {
		return nil, err
	}

	return &Packet{
		Header: h,
		Data:   (*packetBytes)[PacketHeaderLength : PacketHeaderLength+h.DataLength],
	}, nil
}

// NewPacketFromData creates a new Packet from the provided data.
//
// Parameters:
//
// channelNumber: The channel number of the Packet.
// sequenceNumber: The sequence number of the Packet.
// data: A pointer to a byte slice containing the Packet data.
//
// Returns:
//
// A Packet object or an error if the data is nil.
func NewPacketFromData(
	channelNumber uint8,
	sequenceNumber uint8,
	data *[]byte,
) (*Packet, error) {
	// Check if data is nil
	if data == nil {
		return nil, ErrNilDataBuffer
	}

	// Create PacketHeader from data
	h, err := NewPacketHeaderFromData(
		channelNumber,
		sequenceNumber,
		data,
	)
	if err != nil {
		return nil, err
	}

	return &Packet{
		Header: h,
		Data:   *data,
	}, nil
}

// SequenceNumber returns the sequence number of the Packet.
//
// Returns:
//
//	The sequence number as an uint8.
func (p *Packet) SequenceNumber() uint8 {
	return p.Header.SequenceNumber
}

// ReportID returns the report ID of the Packet.
//
// Returns:
//
//	The report ID as an uint8 or an error if the data is too short.
func (p *Packet) ReportID() (uint8, error) {
	if len(p.Data) < 1 {
		return 0, ErrPacketDataTooShort
	}
	return p.Data[0], nil
}

// ChannelNumber returns the channel number of the Packet.
//
// Returns:
//
//	The channel number as an uint8.
func (p *Packet) ChannelNumber() uint8 {
	return p.Header.ChannelNumber
}

// PacketByteCount returns the total byte count of the Packet.
//
// Returns:
//
// The total byte count as an int.
func (p *Packet) PacketByteCount() int {
	return p.Header.PacketByteCount
}

// DataLength returns the data length of the Packet.
//
// Returns:
//
// The data length as an int.
func (p *Packet) DataLength() int {
	return p.Header.DataLength
}

// IsError checks if the Packet indicates an error condition.
//
// Returns:
//
//	True if the Packet is an error, otherwise false.
func (p *Packet) IsError() bool {
	return p.Header.IsError()
}

// String returns a string representation of the Packet for debugging purposes.
//
// Parameters:
//
// isBeingSent: A boolean indicating if the Packet is being sent (true) or received (false).
//
// Returns:
//
//	A string containing the Packet details.
func (p *Packet) String(isBeingSent bool) *string {
	if p == nil || p.Header == nil {
		return nil
	}

	// Derive safe data length
	dataLen := p.Header.DataLength
	if dataLen > len(p.Data) {
		dataLen = len(p.Data)
	} else if dataLen < 0 {
		dataLen = 0
	}

	// Title
	var builder strings.Builder
	if isBeingSent {
		builder.WriteString("********** SENDING PACKET *************")
	} else {
		builder.WriteString("********** RECEIVED PACKET *************")
	}

	// Header section
	builder.WriteString("\n\t HEADER")
	builder.WriteString(fmt.Sprintf("\n\t\t Data Length: %d", dataLen))

	// Channel number
	if int(p.Header.ChannelNumber) < len(Channels) {
		builder.WriteString(
			fmt.Sprintf(
				"\n\t\t Channel: %s (%d)",
				Channels[p.Header.ChannelNumber],
				p.Header.ChannelNumber,
			),
		)
	} else {
		builder.WriteString(
			fmt.Sprintf(
				"\n\t\t Channel: UNKNOWN (%d)",
				p.Header.ChannelNumber,
			),
		)
	}

	// Sequence number
	builder.WriteString(
		fmt.Sprintf(
			"\n\t\t Sequence number: %d",
			p.Header.SequenceNumber,
		),
	)

	// Data section
	builder.WriteString("\n\n\t DATA")

	// Optional report decoding (guard length)
	var reportID uint8
	if dataLen >= 1 {
		reportID = p.Data[0]

		// Get the report type
		reportIDStr := "UNKNOWN"
		channelNumber := p.ChannelNumber()
		switch channelNumber {
		case ChannelSHTPCommand:
			reportIDStr = SHTPCommandsNames[reportID]
		case ChannelExe:
			reportIDStr = ExeCommandsNames[reportID]
		case ChannelControl:
			reportIDStr = ControlCommandsNames[reportID]
		case ChannelInputSensorReports:
			reportIDStr = ControlCommandsNames[reportID]
		}

		builder.WriteString(
			fmt.Sprintf(
				"\n\t\t Report Type: %s (0x%02X)",
				reportIDStr,
				reportID,
			),
		)
	}

	// Additional interpretation (requires at least 6 data bytes)
	if dataLen >= 6 {
		// High report IDs (command responses / meta)
		if isControlReport(reportID) {
			sensorReportType := p.Data[5]
			if name, ok := SHTPCommandsNames[sensorReportType]; ok {
				builder.WriteString(
					fmt.Sprintf(
						"\n\t\t Sensor Report Type: %s (0x%02X)",
						name,
						sensorReportType,
					),
				)
			}
		}

		if reportID == ReportIDGetFeatureResponse || reportID == ReportIDSetFeatureCommand {
			featureID := p.Data[1]
			if name, ok := SHTPCommandsNames[featureID]; ok {
				builder.WriteString(
					fmt.Sprintf(
						"\n\t\t Enabled Feature: %s (0x%02X)",
						name,
						featureID,
					),
				)
			}
		}
	}

	// Iterate only over actual data (exclude header bytes already removed)
	builder.WriteString("\n\t\t Bytes:")
	for idx := 0; idx < dataLen; idx++ {
		packetIdx := idx + PacketHeaderLength // original packet offset including header
		if (packetIdx % PacketHeaderLength) == 0 {
			builder.WriteString(fmt.Sprintf("\n\t\t\t [0x%02X] ", packetIdx))
		}
		builder.WriteString(fmt.Sprintf("0x%02X ", p.Data[idx]))
	}

	builder.WriteString("\n\t *******************************")
	str := builder.String()
	return &str
}

// Buffer returns the byte representation of the Packet.
//
// Returns:
//
// A pointer to a byte slice containing the Packet bytes.
func (p *Packet) Buffer() *[]byte {
	// Initialize packet buffer
	buffer := make([]byte, p.Header.PacketByteCount)

	// Copy header bytes
	headerBuffer := p.Header.Buffer()
	copy(buffer[:PacketHeaderLength], *headerBuffer)

	// Copy data bytes
	copy(buffer[PacketHeaderLength:], p.Data)

	return &buffer
}
