package go_adafruit_bno055

import (
	"time"
)

// ElapsedTime calculates the duration since the provided start time
//
// Parameters:
//
//	startTime: The time at which the measurement started
//
// Returns:
//
//	The elapsed time since startTime
func ElapsedTime(startTime time.Time) time.Duration {
	return time.Now().Sub(startTime)
}
