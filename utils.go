package go_adafruit_bno055

import (
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
func separateBatch(packet packet, reportSlices *[][]interface{}) {
	nextByteIndex := 0
	for nextByteIndex < packet.Header.DataLength {
		reportID := packet.Data[nextByteIndex]
		requiredBytes := ReportLength(reportID)

		unprocessedByteCount := packet.Header.DataLength - nextByteIndex

		if unprocessedByteCount < requiredBytes {
			panic(
				fmt.Sprintf(
					"Unprocessable Batch bytes: %d",
					unprocessedByteCount,
				),
			)
		}

		reportSlice := packet.Data[nextByteIndex : nextByteIndex+requiredBytes]
		*reportSlices = append(
			*reportSlices,
			[]interface{}{reportSlice[0], reportSlice},
		)
		nextByteIndex += requiredBytes
	}
}
