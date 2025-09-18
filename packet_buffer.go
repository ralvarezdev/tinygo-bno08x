package tinygo_bno08x

import (
	tinygoerrors "github.com/ralvarezdev/tinygo-errors"
)

type (
	// DefaultPacketBuffer is a default implementation of the PacketBuffer interface
	DefaultPacketBuffer struct {
		buffer                 []byte
		sequenceNumber         []uint8
		reportsSequenceNumbers [256]uint8
	}
)

// NewDefaultPacketBuffer creates a new DefaultPacketBuffer instance
func NewDefaultPacketBuffer() *DefaultPacketBuffer {
	return &DefaultPacketBuffer{
		buffer:                 make([]byte, PacketBufferSize),
		sequenceNumber:         make([]uint8, MaxChannelNumber),
		reportsSequenceNumbers: [256]uint8{},
	}
}

// GetBuffer returns the buffer
//
// Returns:
//
// A slice of bytes representing the buffer.
func (pb *DefaultPacketBuffer) GetBuffer() []byte {
	return pb.buffer
}

// SetBufferValue sets the buffer value at the specified index
//
// Parameters:
//
//	index: The index in the buffer to set the value.
//	value: The byte value to set at the specified index.
//
// Returns:
//
// An error if the index is out of range, otherwise nil.
func (pb *DefaultPacketBuffer) SetBufferValue(
	index int,
	value byte,
) tinygoerrors.ErrorCode {
	if index < 0 || index >= len(pb.buffer) {
		return ErrorCodeBNO08XPacketBufferIndexOutOfRange
	}
	pb.buffer[index] = value
	return tinygoerrors.ErrorCodeNil
}

// SetBuffer sets the buffer with the provided data slice
//
// Parameters:
//
//	buffer: The byte slice to set as the buffer.
//
// Returns:
//
// An error if the buffer slice is nil or exceeds the buffer size, otherwise nil.
func (pb *DefaultPacketBuffer) SetBuffer(buffer []byte) tinygoerrors.ErrorCode {
	if buffer == nil {
		return ErrorCodeBNO08XNilPacketBuffer
	}
	pb.buffer = buffer
	return tinygoerrors.ErrorCodeNil
}

// ClearBuffer clears the buffer
func (pb *DefaultPacketBuffer) ClearBuffer() {
	for i := range pb.buffer {
		pb.buffer[i] = 0
	}
}

// validateChannelNumber validates the channel number
//
// Parameters:
//
//	channel: The channel number to validate.
//
// Returns:
//
//	An error if the channel number is invalid, otherwise nil.
func (pb *DefaultPacketBuffer) validateChannelNumber(channel uint8) tinygoerrors.ErrorCode {
	if int(channel) < 0 || int(channel) >= len(pb.sequenceNumber) {
		return ErrorCodeBNO08XInvalidChannelNumber
	}
	return tinygoerrors.ErrorCodeNil
}

// IncrementChannelSequenceNumber increments the sequence number for the given channel by the specified amount.
// It wraps at 256.
//
// Parameters:
//
//	channel: The channel for which to increment the sequence number.
//
// Returns:
//
//	The new sequence number for the channel, or an error if the channel is invalid.
func (pb *DefaultPacketBuffer) IncrementChannelSequenceNumber(channel uint8) (
	uint8,
	tinygoerrors.ErrorCode,
) {
	// Validate the channel number
	if err := pb.validateChannelNumber(channel); err != tinygoerrors.ErrorCodeNil {
		return 0, err
	}

	// Increment the sequence number and wrap at 256
	newSequenceNumber := pb.sequenceNumber[int(channel)] + 1
	pb.sequenceNumber[int(channel)] = newSequenceNumber
	return newSequenceNumber, tinygoerrors.ErrorCodeNil
}

// GetChannelSequenceNumber returns the cached sequence number for the given channel.
//
// Parameters:
//
//	channel: The channel for which to get the sequence number.
//
// Returns:
//
//	The cached sequence number for the channel, or -1 if the channel is invalid.
func (pb *DefaultPacketBuffer) GetChannelSequenceNumber(channel uint8) (
	uint8,
	tinygoerrors.ErrorCode,
) {
	// Validate the channel number
	if err := pb.validateChannelNumber(channel); err != tinygoerrors.ErrorCodeNil {
		return 0, err
	}

	// Return the sequence number for the channel
	return pb.sequenceNumber[int(channel)], tinygoerrors.ErrorCodeNil
}

// SetSequenceNumber sets the cached sequence number for the given channel.
//
// Parameters:
//
//	channel: The channel for which to set the sequence number.
//	sequenceNumber: The sequence number to set for the channel.
//
// Returns:
//
// An error if the channel is invalid, otherwise nil.
func (pb *DefaultPacketBuffer) SetSequenceNumber(
	channel uint8,
	sequenceNumber uint8,
) tinygoerrors.ErrorCode {
	// Validate the channel number
	if err := pb.validateChannelNumber(channel); err != tinygoerrors.ErrorCodeNil {
		return err
	}

	// Set the sequence number for the channel
	pb.sequenceNumber[int(channel)] = sequenceNumber
	return tinygoerrors.ErrorCodeNil
}

// IncrementReportSequenceNumber increments the sequence number for the given report ID, wrapping at 256.
//
// Parameters:
//
//	reportID: The ID of the report for which to increment the sequence number.
func (pb *DefaultPacketBuffer) IncrementReportSequenceNumber(reportID uint8) {
	pb.reportsSequenceNumbers[reportID]++
}

// GetReportSequenceNumber returns the current sequence number for the given report ID.
//
// Parameters:
//
//	reportID: The ID of the report for which to get the sequence number.
//
// Returns:
//
//	The current sequence number for the report ID.
func (pb *DefaultPacketBuffer) GetReportSequenceNumber(reportID uint8) uint8 {
	return pb.reportsSequenceNumbers[reportID]
}

// ResetSequenceNumbers resets sequence numbers
func (pb *DefaultPacketBuffer) ResetSequenceNumbers() {
	// Reset the sequence numbers for all channels
	for i := range pb.sequenceNumber {
		pb.sequenceNumber[i] = 0
	}

	// Reset the sequence numbers for all reports
	for reportID := range pb.reportsSequenceNumbers {
		pb.reportsSequenceNumbers[reportID] = 0
	}
}
