package go_adafruit_bno055

/*
SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries

SPDX-License-Identifier: MIT
*/

var (
	// Channels contains the channel IDs and their names
	Channels = map[uint8]string{
		0x0: "SHTP_COMMAND",
		0x1: "EXE",
		0x2: "CONTROL",
		0x3: "INPUT_SENSOR_REPORTS",
		0x4: "WAKE_INPUT_SENSOR_REPORTS",
		0x5: "GYRO_ROTATION_VECTOR",
	}

	// Reports contains the report IDs and their names
	Reports = map[uint8]string{
		0xFB: "BASE_TIMESTAMP",
		0xF2: "COMMAND_REQUEST",
		0xF1: "COMMAND_RESPONSE",
		0xF4: "FRS_READ_REQUEST",
		0xF3: "FRS_READ_RESPONSE",
		0xF6: "FRS_WRITE_DATA",
		0xF7: "FRS_WRITE_REQUEST",
		0xF5: "FRS_WRITE_RESPONSE",
		0xFE: "GET_FEATURE_REQUEST",
		0xFC: "GET_FEATURE_RESPONSE",
		0xFD: "SET_FEATURE_COMMAND",
		0xFA: "TIMESTAMP_REBASE",
		0x01: "ACCELEROMETER",
		0x29: "ARVR_STABILIZED_GAME_ROTATION_VECTOR",
		0x28: "ARVR_STABILIZED_ROTATION_VECTOR",
		0x22: "CIRCLE_DETECTOR",
		0x1A: "FLIP_DETECTOR",
		0x08: "GAME_ROTATION_VECTOR",
		0x09: "GEOMAGNETIC_ROTATION_VECTOR",
		0x06: "GRAVITY",
		0x02: "GYROSCOPE",
		0x04: "LINEAR_ACCELERATION",
		0x03: "MAGNETIC_FIELD",
		0x1E: "PERSONAL_ACTIVITY_CLASSIFIER",
		0x1B: "PICKUP_DETECTOR",
		0x21: "POCKET_DETECTOR",
		0xF9: "PRODUCT_ID_REQUEST",
		0xF8: "PRODUCT_ID_RESPONSE",
		0x14: "RAW_ACCELEROMETER",
		0x15: "RAW_GYROSCOPE",
		0x16: "RAW_MAGNETOMETER",
		0x05: "ROTATION_VECTOR",
		0x17: "SAR",
		0x19: "SHAKE_DETECTOR",
		0x12: "SIGNIFICANT_MOTION",
		0x1F: "SLEEP_DETECTOR",
		0x13: "STABILITY_CLASSIFIER",
		0x1C: "STABILITY_DETECTOR",
		0x11: "STEP_COUNTER",
		0x18: "STEP_DETECTOR",
		0x10: "TAP_DETECTOR",
		0x20: "TILT_DETECTOR",
		0x07: "UNCALIBRATED_GYROSCOPE",
		0x0F: "UNCALIBRATED_MAGNETIC_FIELD",
	}
)

/*
const (
	// ReportIDRawUncalibratedGyroscope is the report ID for uncalibrated gyroscope (rad/s).
	ReportIDRawUncalibratedGyroscope uint8 = 0x07 // 16

	// ReportIDUncalibratedMagneticField is the report ID for magnetic field uncalibrated (in µTesla).
	// The magnetic field measurement without hard-iron offset applied, the hard-iron
	// estimate is provided as a separate parameter.
	ReportIDUncalibratedMagneticField uint8 = 0x0F

	// ReportIDARVRStabilizedGameRotationVector is the reportID for AR/VR Stabilized Game Rotation vector
	ReportIDARVRStabilizedGameRotationVector uint8 = 0x29

	// ReportIDARVRStabilizedRotationVector is the reportID for AR/VR Stabilized Rotation Vector
	ReportIDARVRStabilizedRotationVector uint8 = 0x28

	// ReportIDTapDetector is the report ID for the tap detector
	ReportIDTapDetector       uint8 = 0x10

	// ReportIDSignificantMotion is the report ID for the significant motion sensor
	ReportIDSignificantMotion uint8 = 0x12

	// ReportIDSar is the report ID for the SAR (Specific Absorption Rate) sensor
	ReportIDSar               uint8 = 0x17

	// ReportIDStepDetector is the report ID for the step detector
	ReportIDStepDetector      uint8 = 0x18

	// ReportIDFlipDetector is the report ID for the flip detector
	ReportIDFlipDetector      uint8 = 0x1A

	// ReportIDPickupDetector is the report ID for the pickup detector
	ReportIDPickupDetector    uint8 = 0x1B

	// ReportIDStabilityDetector is the report ID for the stability detector
	ReportIDStabilityDetector uint8 = 0x1C

	// ReportIDSleepDetector is the report ID for the sleep detector
	ReportIDSleepDetector     uint8 = 0x1F

	// ReportIDTiltDetector is the report ID for the tilt detector
	ReportIDTiltDetector      uint8 = 0x20

	// ReportIDPocketDetector is the report ID for the pocket detector
	ReportIDPocketDetector    uint8 = 0x21

	// ReportIDCircleDetector is the report ID for the circle detector
	ReportIDCircleDetector    uint8 = 0x22
)
*/
