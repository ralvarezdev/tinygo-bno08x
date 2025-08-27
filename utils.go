//go:build tinygo && (rp2040 || rp2350)

package go_bno08x

import (
	"errors"
	"fmt"
	"math"
	"time"

	"machine"
)

// HardwareReset performs a hardware reset of the BNO08X sensor to an initial unconfigured state.
//
// Parameters:
//
// reset: The machine.Pin used to perform the hardware reset.
// debugger: An optional Debugger for logging debug information during the reset process.
func HardwareReset(resetPin machine.Pin, debugger Debugger) {
	if debugger != nil {
		debugger.Debug("Hardware resetting...")
	}

	// Configure the reset pin as output
	resetPin.Configure(machine.PinConfig{Mode: machine.PinOutput})

	resetPin.High()
	time.Sleep(10 * time.Millisecond)

	resetPin.Low()
	time.Sleep(10 * time.Millisecond)

	resetPin.High()
	time.Sleep(10 * time.Millisecond)
}

// separateBatch takes a Packet and separates it into individual reports, appending them to the provided reports.
//
// Parameters:
//
// packet: The Packet to separate into reports.
// reports: A pointer to a slice of slices where the separated reports will be appended.
//
// Returns:
//
// An error if the Packet cannot be processed due to insufficient bytes or other issues.
func separateBatch(packet *Packet, reports *[]*report) error {
	// Check if the packet is nil
	if packet == nil {
		return ErrNilPacket
	}

	// Check if the packet header is nil
	if packet.Header == nil {
		return ErrNilPacketHeader
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
					"unprocessable batch bytes: %d",
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

// QuaternionToEulerDegrees converts the quaternion representation of orientation to Euler angles (roll, pitch, yaw) in degrees.
//
// Returns:
//
// A tuple of three float64 values representing the roll, pitch, and yaw angles in degrees, or an error if the input is nil.
func QuaternionToEulerDegrees(rotationVector *[4]float64) (*[3]float64, error) {
	// Check if the rotation vector is nil
	if rotationVector == nil {
		return nil, ErrNilRotationVector
	}

	// Get the quaternion components
	x := rotationVector[0]
	y := rotationVector[1]
	z := rotationVector[2]
	w := rotationVector[3]

	// Roll (X axis)
	sinRollCosPitch := 2 * (w*x + y*z)
	cosRollCosPitch := 1 - 2*(x*x+y*y)
	roll := math.Atan2(sinRollCosPitch, cosRollCosPitch)

	// Pitch (Y axis)
	sinPitch := 2 * (w*y - z*x)
	var pitch float64
	if sinPitch >= 1 {
		pitch = math.Pi / 2
	} else if sinPitch <= -1 {
		pitch = -math.Pi / 2
	} else {
		pitch = math.Asin(sinPitch)
	}

	// Yaw (Z axis)
	sinYawCosPitch := 2 * (w*z + x*y)
	cosYawCosPitch := 1 - 2*(y*y+z*z)
	yaw := math.Atan2(sinYawCosPitch, cosYawCosPitch)

	return &[3]float64{
		roll * 180 / math.Pi,
		pitch * 180 / math.Pi,
		yaw * 180 / math.Pi,
	}, nil
}
