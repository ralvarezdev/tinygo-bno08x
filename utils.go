//go:build tinygo && (rp2040 || rp2350)

package go_adafruit_bno08x

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

// separateBatch takes a Packet and separates it into individual reports, appending them to the provided reports.
//
// Parameters:
//
//	Packet: The Packet to separate into reports.
//	reports: A pointer to a slice of slices where the separated reports will be appended.
//
// Returns:
//
// An error if the Packet cannot be processed due to insufficient bytes or other issues.
func separateBatch(packet *Packet, reports *[]*report) error {
	// Check if the Packet is nil
	if packet == nil {
		return ErrNilPacket
	}

	// Ensure the Packet has a valid header
	nextByteIndex := 0
	for nextByteIndex < packet.Header.DataLength {
		// Check if there are enough bytes left in the Packet to read the report ID
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

		reportBytes := packet.Data[nextByteIndex : nextByteIndex+requiredBytes]
		report, err := newReport(reportID, &reportBytes)
		if err != nil {
			return fmt.Errorf(
				"failed to create report from bytes: %w",
				err,
			)
		}

		// Append the new report to the reports
		*reports = append(
			*reports,
			report,
		)
		nextByteIndex += requiredBytes
	}
	return nil
}
