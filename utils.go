package go_adafruit_bno055

import (
	"errors"
	"fmt"
	"time"
)

// elapsedTime calculates the duration since the provided start time
//
// Parameters:
//
//	startTime: The time at which the measurement started
//
// Returns:
//
//	The elapsed time since startTime
func elapsedTime(startTime time.Time) time.Duration {
	return time.Now().Sub(startTime)
}

// separateBatch takes a packet and separates it into individual reports, appending them to the provided reportSlices.
//
// Parameters:
//
//	packet: The packet to separate into reports.
//	reportSlices: A pointer to a slice of slices where the separated reports will be appended.
//
// Returns:
//
// An error if the packet cannot be processed due to insufficient bytes or other issues.
func separateBatch(packet *packet, reportSlices *[]*report) error {
	// Check if the packet is nil
	if packet == nil {
		return ErrNilPacket
	}

	// Ensure the packet has a valid header
	nextByteIndex := 0
	for nextByteIndex < packet.Header.DataLength {
		// Check if there are enough bytes left in the packet to read the report ID
		reportID := packet.Data[nextByteIndex]
		requiredBytes := reportLength(reportID)
		unprocessedByteCount := packet.Header.DataLength - nextByteIndex

		if unprocessedByteCount < requiredBytes {
			return errors.New(
				fmt.Sprintf(
					"unprocessable Batch bytes: %d",
					unprocessedByteCount,
				),
			)
		}

		reportSlice := packet.Data[nextByteIndex : nextByteIndex+requiredBytes]
		report, err := newReport(reportID, &reportSlice)
		if err != nil {
			return fmt.Errorf(
				"failed to create report from bytes: %w",
				err,
			)
		}

		// Append the new report to the reportSlices
		*reportSlices = append(
			*reportSlices,
			report,
		)
		nextByteIndex += requiredBytes
	}
	return nil
}
