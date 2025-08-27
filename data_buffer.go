//go:build tinygo && (rp2040 || rp2350)

package go_bno08x

type (
	// DataBuffer is an interface for managing data buffers
	DataBuffer interface {
		GetData() *[]byte
		SetData(data *[]byte)
		ClearData()
		UpdateSequenceNumber(newPacket *Packet) error
		IncrementChannelSequenceNumber(channel uint8) (uint8, error)
		GetSequenceNumber(channel uint8) (uint8, error)
		SetSequenceNumber(channel uint8, sequenceNumber uint8) error
		IncrementReportSequenceNumber(reportID uint8)
		GetReportSequenceNumber(reportID uint8) uint8
		ResetSequenceNumbers()
	}

	// DefaultDataBuffer is a default implementation of the DataBuffer interface
	DefaultDataBuffer struct {
		data                    *[]byte
		sequenceNumber          []uint8
		twoEndedSequenceNumbers map[uint8]uint8
	}
)

// NewDefaultDataBuffer creates a new DefaultDataBuffer instance
func NewDefaultDataBuffer() *DefaultDataBuffer {
	// Initialize the data buffer with a size of DataBufferSize
	data := make([]byte, DataBufferSize)

	return &DefaultDataBuffer{
		data:                    &data,
		sequenceNumber:          make([]uint8, 6), // Assuming 6 channels
		twoEndedSequenceNumbers: make(map[uint8]uint8),
	}
}

// GetData returns the data buffer
//
// Returns:
//
// A pointer to the byte slice representing the data buffer.
func (db *DefaultDataBuffer) GetData() *[]byte {
	return db.data
}

// SetData sets the data buffer
//
// Parameters:
//
//	data: A pointer to the byte slice to set as the data buffer. If nil, the data buffer is cleared.
func (db *DefaultDataBuffer) SetData(data *[]byte) {
	if data == nil {
		db.ClearData()
	} else {
		db.data = data
	}
}

// ClearData clears the data buffer
func (db *DefaultDataBuffer) ClearData() {
	db.data = new([]byte)
	*db.data = make([]byte, DataBufferSize)
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
	current, ok := db.twoEndedSequenceNumbers[reportID]
	if !ok {
		current = 0
	}
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
	sequenceNumber, ok := db.twoEndedSequenceNumbers[reportID]
	if !ok {
		db.sequenceNumber[reportID] = 0
		return 0
	}
	return sequenceNumber
}

// ResetSequenceNumbers resets sequence numbers
func (db *DefaultDataBuffer) ResetSequenceNumbers() {
	for i := range db.sequenceNumber {
		db.sequenceNumber[i] = 0
	}
	db.twoEndedSequenceNumbers = make(map[uint8]uint8)
}
