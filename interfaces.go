package tinygo_bno08x

import (
	tinygoerrors "github.com/ralvarezdev/tinygo-errors"
)

type (
	// PacketBuffer is an interface for managing packet buffers
	PacketBuffer interface {
		GetBuffer() []byte
		SetBufferValue(index int, value byte) tinygoerrors.ErrorCode
		SetBuffer(data []byte) tinygoerrors.ErrorCode
		ClearBuffer()
		IncrementChannelSequenceNumber(channel uint8) (
			uint8,
			tinygoerrors.ErrorCode,
		)
		GetChannelSequenceNumber(channel uint8) (uint8, tinygoerrors.ErrorCode)
		IncrementReportSequenceNumber(reportID uint8)
		GetReportSequenceNumber(reportID uint8) uint8
		ResetSequenceNumbers()
	}

	// PacketReader is an interface for reading packets from the BNO08x sensor
	PacketReader interface {
		ReadPacket() (Packet, tinygoerrors.ErrorCode)
		IsAvailableToRead() bool
	}

	// PacketWriter is an interface for writing packets to the BNO08x sensor
	PacketWriter interface {
		SendPacket(channel uint8, data []byte) (uint8, tinygoerrors.ErrorCode)
	}
)
