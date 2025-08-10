package go_adafruit_bno055

type (
	// DataBuffer is an interface for managing data buffers
	DataBuffer interface {
		GetData() *[]byte
		SetData(data *[]byte)
		Clear()
		UpdateSequenceNumber(newPacket *Packet) error
		IncrementChannelSequenceNumber(channel uint8) (uint8, error)
		GetSequenceNumber(channel uint8) (uint8, error)
		SetSequenceNumber(channel uint8, sequenceNumber uint8) error
		IncrementReportSequenceNumber(reportID uint8)
		GetReportSequenceNumber(reportID uint8) int
	}

	// DefaultDataBuffer is a default implementation of the DataBuffer interface
	DefaultDataBuffer struct {
		data                    *[]byte
		sequenceNumber          []uint8
		twoEndedSequenceNumbers map[uint8]uint8
	}
)

// newDefaultDataBuffer creates a new DefaultDataBuffer instance
func newDefaultDataBuffer() *DefaultDataBuffer {
	return &DefaultDataBuffer{
		data:                    &[]byte{},
		sequenceNumber:          make([]uint8, 6), // Assuming 6 channels
		twoEndedSequenceNumbers: make(map[uint8]uint8),
	}
}

// GetData returns the data buffer
func (db *DefaultDataBuffer) GetData() *[]byte {
	return db.data
}

// SetData sets the data buffer
func (db *DefaultDataBuffer) SetData(data *[]byte) {
	if data == nil {
		db.data = &[]byte{}
	} else {
		db.data = data
	}
}

// Clear clears the data buffer
func (db *DefaultDataBuffer) Clear() {
	db.data = &[]byte{}
}

// UpdateSequenceNumber updates the cached sequence number for the given channel using the provided Packet.
//
// Parameters:
//
//	NewPacket: A pointer to the Packet containing the channel and sequence number.
//
// Returns:
//
//	An error if the sequence number could not be updated, otherwise nil.
func (db *DefaultDataBuffer) UpdateSequenceNumber(newPacket *Packet) error {
	// Check if the NewPacket is nil
	if newPacket == nil {
		return ErrNilPacket
	}

	channel := newPacket.ChannelNumber()
	seq := newPacket.Header.SequenceNumber
	db.sequenceNumber[int(channel)] = seq
	return nil
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
func (db *DefaultDataBuffer) IncrementChannelSequenceNumber(channel uint8) (
	uint8,
	error,
) {
	if int(channel) < 0 || int(channel) >= len(db.sequenceNumber) {
		return 0, ErrInvalidChannel
	}
	current := db.sequenceNumber[int(channel)]
	newSequenceNumber := current + 1
	db.sequenceNumber[int(channel)] = newSequenceNumber
	return newSequenceNumber, nil
}

// GetSequenceNumber returns the cached sequence number for the given channel.
//
// Parameters:
//
//	channel: The channel for which to get the sequence number.
//
// Returns:
//
//	The cached sequence number for the channel, or -1 if the channel is invalid.
func (db *DefaultDataBuffer) GetSequenceNumber(channel uint8) (uint8, error) {
	if int(channel) < 0 || int(channel) >= len(db.sequenceNumber) {
		return 0, ErrInvalidChannel
	}
	return db.sequenceNumber[int(channel)], nil
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
func (db *DefaultDataBuffer) SetSequenceNumber(
	channel uint8,
	sequenceNumber uint8,
) error {
	if int(channel) < 0 || int(channel) >= len(db.sequenceNumber) {
		return ErrInvalidChannel
	}
	db.sequenceNumber[int(channel)] = sequenceNumber
	return nil
}

// IncrementReportSequenceNumber increments the sequence number for the given report ID, wrapping at 256.
//
// Parameters:
//
//	reportID: The ID of the report for which to increment the sequence number.
func (db *DefaultDataBuffer) IncrementReportSequenceNumber(reportID uint8) {
	current := db.twoEndedSequenceNumbers[reportID]
	db.twoEndedSequenceNumbers[reportID] = current + 1
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
func (db *DefaultDataBuffer) GetReportSequenceNumber(reportID uint8) uint8 {
	return db.twoEndedSequenceNumbers[reportID]
}
