//go:build tinygo && (rp2040 || rp2350)

package tinygo_bno08x

import (
	tinygotypes "github.com/ralvarezdev/tinygo-types"
)

type (
	// ReportAccuracyStatus is an enumeration of accuracy status values
	ReportAccuracyStatus uint8

	// ReportClassification is an enumeration of classification keys
	ReportClassification uint8

	// ReportActivity is an enumeration of activity keys
	ReportActivity uint8

	// ReportStabilityClassification is an enumeration of stability classification keys
	ReportStabilityClassification uint8

	// Mode is an enumeration of operation modes
	Mode uint8
)

const (
	// ReportClassificationsNumber is the number of classification keys
	ReportClassificationsNumber = 10
)

const (
	ReportAccuracyStatusUnreliable ReportAccuracyStatus = iota
	ReportAccuracyStatusLow
	ReportAccuracyStatusMedium
	ReportAccuracyStatusHigh
	ReportAccuracyStatusNil
)

const (
	ReportClassificationTilting ReportClassification = iota
	ReportClassificationOnStairs
	ReportClassificationOnFoot
	ReportClassificationOther
	ReportClassificationOnBicycle
	ReportClassificationStill
	ReportClassificationWalking
	ReportClassificationUnknown
	ReportClassificationRunning
	ReportClassificationInVehicle
	ReportClassificationNil
)

const (
	ReportActivityUnknown ReportActivity = iota
	ReportActivityInVehicle
	ReportActivityOnBicycle
	ReportActivityOnFoot
	ReportActivityStill
	ReportActivityTilting
	ReportActivityWalking
	ReportActivityRunning
	ReportActivityOnStairs
	ReportActivityNil
)

const (
	ReportStabilityClassificationUnknown ReportStabilityClassification = iota
	ReportStabilityClassificationOnTable
	ReportStabilityClassificationStationary
	ReportStabilityClassificationStable
	ReportStabilityClassificationInMotion
	ReportStabilityClassificationNil
)

const (
	ModeNil Mode = iota
	I2CMode 
	UARTMode
	UARTRVCMode
	SPIMode
)

// ReportAccuracyStatusFromUint8 returns the ReportAccuracyStatus enum based on a given uint8 value
//
// Parameters:
//
// value: The uint8 value to search on ReportAccuracyStatuses
//
// Returns:
//
// The ReportAccuracyStatus enum value, or an error if the key wasn't found for the given value
func ReportAccuracyStatusFromUint8(value uint8) (ReportAccuracyStatus, tinygotypes.ErrorCode) {
	switch ReportAccuracyStatus(value) {
	case ReportAccuracyStatusUnreliable:
		return ReportAccuracyStatusUnreliable, tinygotypes.ErrorCodeNil
	case ReportAccuracyStatusLow:
		return ReportAccuracyStatusLow, tinygotypes.ErrorCodeNil
	case ReportAccuracyStatusMedium:
		return ReportAccuracyStatusMedium, tinygotypes.ErrorCodeNil
	case ReportAccuracyStatusHigh:
		return ReportAccuracyStatusHigh, tinygotypes.ErrorCodeNil
	default:
		return ReportAccuracyStatusNil, ErrorCodeBNO08XInvalidReportAccuracyStatusUint8
	}
}

// ReportActivityFromUint8 returns the ReportActivity enum based on a given uint8 value
//
// Parameters:
//
// value: The uint8 value to search on ReportActivities
//
// Returns:
//
// The ReportActivity enum value, or an error if the key wasn't found for the given value
func ReportActivityFromUint8(value uint8) (ReportActivity, tinygotypes.ErrorCode) {
	switch ReportActivity(value) {
	case ReportActivityUnknown:
		return ReportActivityUnknown, tinygotypes.ErrorCodeNil
	case ReportActivityInVehicle:
		return ReportActivityInVehicle, tinygotypes.ErrorCodeNil
	case ReportActivityOnBicycle:
		return ReportActivityOnBicycle, tinygotypes.ErrorCodeNil
	case ReportActivityOnFoot:
		return ReportActivityOnFoot, tinygotypes.ErrorCodeNil
	case ReportActivityStill:
		return ReportActivityStill, tinygotypes.ErrorCodeNil	
	case ReportActivityTilting:
		return ReportActivityTilting, tinygotypes.ErrorCodeNil
	case ReportActivityWalking:
		return ReportActivityWalking, tinygotypes.ErrorCodeNil
	case ReportActivityRunning:
		return ReportActivityRunning, tinygotypes.ErrorCodeNil
	case ReportActivityOnStairs:
		return ReportActivityOnStairs, tinygotypes.ErrorCodeNil
	default:
		return ReportActivityNil, ErrorCodeBNO08XInvalidReportActivityUint8
	}
}

// ReportStabilityClassificationFromUint8 returns the ReportStabilityClassification enum based on a given uint8 value
//
// Parameters:
//
// value: The uint8 value to search on ReportStabilityClassifications
//
// Returns:
//
// The ReportStabilityClassification enum value, or an error if the key wasn't found for the given value
func ReportStabilityClassificationFromUint8(value uint8) (ReportStabilityClassification, tinygotypes.ErrorCode) {
	switch ReportStabilityClassification(value) {
	case ReportStabilityClassificationUnknown:
		return ReportStabilityClassificationUnknown, tinygotypes.ErrorCodeNil
	case ReportStabilityClassificationOnTable:
		return ReportStabilityClassificationOnTable, tinygotypes.ErrorCodeNil
	case ReportStabilityClassificationStationary:
		return ReportStabilityClassificationStationary, tinygotypes.ErrorCodeNil
	case ReportStabilityClassificationStable:
		return ReportStabilityClassificationStable, tinygotypes.ErrorCodeNil
	case ReportStabilityClassificationInMotion:
		return ReportStabilityClassificationInMotion, tinygotypes.ErrorCodeNil
	default:
		return ReportStabilityClassificationNil, ErrorCodeBNO08XInvalidReportStabilityClassificationUint8
	}
}