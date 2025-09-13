//go:build tinygo && (rp2040 || rp2350)

package tinygo_bno08x

import (
	tinygotypes "github.com/ralvarezdev/tinygo-types"
)

type (
	// PacketBuffer is an interface for managing packet buffers
	PacketBuffer interface {
		GetBuffer() []byte
		SetBufferValue(index int, value byte) tinygotypes.ErrorCode
		SetBuffer(data []byte) tinygotypes.ErrorCode
		ClearBuffer()
		IncrementChannelSequenceNumber(channel uint8) (uint8, tinygotypes.ErrorCode)
		GetChannelSequenceNumber(channel uint8) (uint8, tinygotypes.ErrorCode)
		IncrementReportSequenceNumber(reportID uint8)
		GetReportSequenceNumber(reportID uint8) uint8
		ResetSequenceNumbers()
	}

	// PacketReader is an interface for reading packets from the BNO08x sensor
	PacketReader interface {
		ReadPacket() (Packet, tinygotypes.ErrorCode)
		IsAvailableToRead() bool
	}

	// PacketWriter is an interface for writing packets to the BNO08x sensor
	PacketWriter interface {
		SendPacket(channel uint8, data []byte) (uint8, tinygotypes.ErrorCode)
	}
)
