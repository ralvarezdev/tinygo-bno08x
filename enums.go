package tinygo_bno08x

import (
	tinygoerrors "github.com/ralvarezdev/tinygo-errors"
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
func ReportAccuracyStatusFromUint8(value uint8) (ReportAccuracyStatus, tinygoerrors.ErrorCode) {
	switch ReportAccuracyStatus(value) {
	case ReportAccuracyStatusUnreliable:
		return ReportAccuracyStatusUnreliable, tinygoerrors.ErrorCodeNil
	case ReportAccuracyStatusLow:
		return ReportAccuracyStatusLow, tinygoerrors.ErrorCodeNil
	case ReportAccuracyStatusMedium:
		return ReportAccuracyStatusMedium, tinygoerrors.ErrorCodeNil
	case ReportAccuracyStatusHigh:
		return ReportAccuracyStatusHigh, tinygoerrors.ErrorCodeNil
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
func ReportActivityFromUint8(value uint8) (ReportActivity, tinygoerrors.ErrorCode) {
	switch ReportActivity(value) {
	case ReportActivityUnknown:
		return ReportActivityUnknown, tinygoerrors.ErrorCodeNil
	case ReportActivityInVehicle:
		return ReportActivityInVehicle, tinygoerrors.ErrorCodeNil
	case ReportActivityOnBicycle:
		return ReportActivityOnBicycle, tinygoerrors.ErrorCodeNil
	case ReportActivityOnFoot:
		return ReportActivityOnFoot, tinygoerrors.ErrorCodeNil
	case ReportActivityStill:
		return ReportActivityStill, tinygoerrors.ErrorCodeNil	
	case ReportActivityTilting:
		return ReportActivityTilting, tinygoerrors.ErrorCodeNil
	case ReportActivityWalking:
		return ReportActivityWalking, tinygoerrors.ErrorCodeNil
	case ReportActivityRunning:
		return ReportActivityRunning, tinygoerrors.ErrorCodeNil
	case ReportActivityOnStairs:
		return ReportActivityOnStairs, tinygoerrors.ErrorCodeNil
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
func ReportStabilityClassificationFromUint8(value uint8) (ReportStabilityClassification, tinygoerrors.ErrorCode) {
	switch ReportStabilityClassification(value) {
	case ReportStabilityClassificationUnknown:
		return ReportStabilityClassificationUnknown, tinygoerrors.ErrorCodeNil
	case ReportStabilityClassificationOnTable:
		return ReportStabilityClassificationOnTable, tinygoerrors.ErrorCodeNil
	case ReportStabilityClassificationStationary:
		return ReportStabilityClassificationStationary, tinygoerrors.ErrorCodeNil
	case ReportStabilityClassificationStable:
		return ReportStabilityClassificationStable, tinygoerrors.ErrorCodeNil
	case ReportStabilityClassificationInMotion:
		return ReportStabilityClassificationInMotion, tinygoerrors.ErrorCodeNil
	default:
		return ReportStabilityClassificationNil, ErrorCodeBNO08XInvalidReportStabilityClassificationUint8
	}
}