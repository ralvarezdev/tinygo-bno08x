package go_adafruit_bno055

import (
	"encoding/binary"
	"fmt"
)

type (
	// packetHeader represents the header of a BNO08x packet
	packetHeader struct {
		ChannelNumber   uint8
		SequenceNumber  uint8
		DataLength      int
		PacketByteCount int
	}

	// packet represents a BNO08x packet
	packet struct {
		Header *packetHeader
		Data   []byte
	}
)

// newPacketHeader creates a packetHeader from a given buffer.
//
// Parameters:
//
//	packetBytes: A pointer to a byte slice containing the packet data.
//
// Returns:
//
//	A packetHeader object or an error if the buffer is too short.
func newPacketHeader(packetBytes *[]byte) (*packetHeader, error) {
	// Check if the provided packetBytes is nil
	if packetBytes == nil {
		return nil, ErrNilPacketBytes
	}

	// Ensure the buffer is at least 4 bytes long to read the header
	if len(*packetBytes) < 4 {
		return nil, ErrBufferTooShortForHeader
	}

	packetByteCount := binary.LittleEndian.Uint16((*packetBytes)[0:2])
	packetByteCount &= ^uint16(0x8000)
	channelNumber := (*packetBytes)[2]
	sequenceNumber := (*packetBytes)[3]
	dataLength := int(packetByteCount) - 4
	if dataLength < 0 {
		dataLength = 0
	}
	return &packetHeader{
		ChannelNumber:   channelNumber,
		SequenceNumber:  sequenceNumber,
		DataLength:      dataLength,
		PacketByteCount: int(packetByteCount),
	}, nil
}

// IsError checks if the provided packetHeader indicates an error condition.
//
// Parameters:
//
//	header: The packetHeader to check.
//
// Returns:
//
//	True if the header indicates an error, otherwise false.
func (header *packetHeader) IsError() bool {
	// Check if the channel number is greater than 5
	if header.ChannelNumber > 5 {
		return true
	}
	// Check if the packet byte count and sequence number indicate an error
	if header.PacketByteCount == 0xFFFF && header.SequenceNumber == 0xFF {
		return true
	}
	return false
}

// newPacket creates a new packet from the provided packet bytes.
//
// Parameters:
//
//	packetBytes: A pointer to a byte slice containing the packet data.
//
// Returns:
//
//	A packet object or an error if the packet header could not be created.
func newPacket(packetBytes *[]byte) (*packet, error) {
	// Check if the provided packetBytes is nil
	if packetBytes == nil {
		return nil, ErrNilPacketBytes
	}

	// Create a new packetHeader from the packet bytes
	header, err := newPacketHeader(packetBytes)
	if err != nil {
		return nil, err
	}

	return &packet{
		Header: header,
		Data:   (*packetBytes)[BnoHeaderLen : BnoHeaderLen+header.DataLength],
	}, nil
}

// ReportID returns the report ID of the packet.
//
// Returns:
//
//	The report ID as an uint8 or an error if the data is too short.
func (p *packet) ReportID() (uint8, error) {
	if len(p.Data) < 1 {
		return 0, ErrPacketDataTooShort
	}
	return p.Data[0], nil
}

// ChannelNumber returns the channel number of the packet.
//
// Returns:
//
//	The channel number as an uint8.
func (p *packet) ChannelNumber() uint8 {
	return p.Header.ChannelNumber
}

// IsError checks if the packet indicates an error condition.
//
// Returns:
//
//	True if the packet is an error, otherwise false.
func (p *packet) IsError() bool {
	return p.Header.IsError()
}

// String returns a string representation of the packet for debugging purposes.
//
// Returns:
//
//	A string containing the packet details.
func (p *packet) String() *string {
	outputStr := "\n\t\t********** packet *************\n"
	outputStr += "DBG::\t\t HEADER:\n"

	outputStr += fmt.Sprintf("DBG::\t\t Data Length: %d\n", p.Header.DataLength)
	outputStr += fmt.Sprintf(
		"DBG::\t\t Channel: %s (%d)\n",
		Channels[p.Header.ChannelNumber],
		p.Header.ChannelNumber,
	)

	channelNumbers := []uint8{
		BnoChannelCONTROL,
		BnoChannelInputSensorReports,
	}
	for _, channelNumber := range channelNumbers {
		if p.Header.ChannelNumber != channelNumber {
			continue
		}

		packetReportID, err := p.ReportID()
		if err != nil {
			continue
		}

		if _, ok := Reports[packetReportID]; !ok {
			outputStr += fmt.Sprintf(
				"DBG::\t\t \t** UNKNOWN Report Type **: %s\n",
				string(packetReportID),
			)
		} else {
			outputStr += fmt.Sprintf(
				"DBG::\t\t \tReport Type: %s (0x%x)\n",
				Reports[packetReportID],
				packetReportID,
			)
		}

		if packetReportID > 0xF0 && len(p.Data) >= 6 {
			if _, ok := Reports[p.Data[5]]; ok {
				outputStr += fmt.Sprintf(
					"DBG::\t\t \tSensor Report Type: %s(%s)\n",
					Reports[p.Data[5]],
					string(p.Data[5]),
				)
			}
		}

		if packetReportID == 0xFC && len(p.Data) >= 6 {
			if _, ok := Reports[p.Data[1]]; ok {
				outputStr += fmt.Sprintf(
					"DBG::\t\t \tEnabled Feature: %s (0x%x)\n",
					Reports[p.Data[1]],
					p.Data[5],
				)
			}
		}
	}

	outputStr += fmt.Sprintf(
		"DBG::\t\t Sequence number: %s\n\n",
		string(p.Header.SequenceNumber),
	)
	outputStr += "DBG::\t\t Data:"

	for idx, packetByte := range p.Data[:p.Header.PacketByteCount] {
		packetIdx := idx + 4
		if (packetIdx % 4) == 0 {
			outputStr += fmt.Sprintf("\nDBG::\t\t[0x%02X] ", packetIdx)
		}
		outputStr += fmt.Sprintf("0x%02X ", packetByte)
	}
	outputStr += "\n"
	outputStr += "\t\t*******************************\n"
	return &outputStr
}
