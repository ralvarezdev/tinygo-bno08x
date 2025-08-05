package go_adafruit_bno055

// SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
//
// SPDX-License-Identifier: MIT
// A collection of dictionaries for better debug messages

// TODO: Remove for size before release

// Channels contains the channel IDs and their names
var Channels = map[uint8]string{
	0x0: "SHTP_COMMAND",
	0x1: "EXE",
	0x2: "CONTROL",
	0x3: "INPUT_SENSOR_REPORTS",
	0x4: "WAKE_INPUT_SENSOR_REPORTS",
	0x5: "GYRO_ROTATION_VECTOR",
}

// Reports contains the report IDs and their names
var Reports = map[uint8]string{
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

/*
// BnoReportGravity is used for gravity (m/s2)
const BnoReportGravity uint8 = 0x06 // 10 bytes

// BnoReportRawAccelerometer is used for raw uncalibrated accelerometer data (ADC units). Used for testing
const BnoReportRawAccelerometer uint8 = 0x14 // 16

// BnoReportRawUncalibratedGyroscope is used for uncalibrated gyroscope (rad/s).
const BnoReportRawUncalibratedGyroscope uint8 = 0x07 // 16

// BnoReportRawGyroscope is used for raw uncalibrated gyroscope (ADC units).
const BnoReportRawGyroscope uint8 = 0x15 // 16

// BnoReportUncalibratedMagneticField is used for magnetic field uncalibrated (in µTesla).
// The magnetic field measurement without hard-iron offset applied, the hard-iron
// estimate is provided as a separate parameter.
const BnoReportUncalibratedMagneticField uint8 = 0x0F

// BnoReportRawMagnetometer is used for raw magnetic field measurement (in ADC units).
// Direct data from the magnetometer. Used for testing.
const BnoReportRawMagnetometer uint8 = 0x16 // 16

// BnoReportGeomagneticRotationVector is used for Geomagnetic Rotation Vector
const BnoReportGeomagneticRotationVector uint8 = 0x09

// BnoReportGameRotationVector is used for Game Rotation Vector
const BnoReportGameRotationVector uint8 = 0x08

// BnoReportARVRStabilizedGameRotationVector is used for AR/VR Stabilized Game Rotation vector
const BnoReportARVRStabilizedGameRotationVector uint8 = 0x29

// BnoReportARVRStabilizedRotationVector is used for AR/VR Stabilized Rotation Vector
const BnoReportARVRStabilizedRotationVector uint8 = 0x28
*/

// Gyro rotation Vector
// Gyro rotation Vector Prediction

/*
const BnoReportTapDetector uint8 = 0x10
const BnoReportStepCounter uint8 = 0x11
const BnoReportSignificantMotion uint8 = 0x12
*/

/*
const BnoReportSar uint8 = 0x17
const BnoReportStepDetector uint8 = 0x18
const BnoReportShakeDetector uint8 = 0x19
const BnoReportFlipDetector uint8 = 0x1A
const BnoReportPickupDetector uint8 = 0x1B
const BnoReportStabilityDetector uint8 = 0x1C
const BnoReportSleepDetector uint8 = 0x1F
const BnoReportTiltDetector uint8 = 0x20
const BnoReportPocketDetector uint8 = 0x21
const BnoReportCircleDetector uint8 = 0x22
*/

/*
Reset reasons from ID Report response:
0 – Not Applicable
1 – Power On Reset
2 – Internal System Reset
3 – Watchdog Timeout
4 – External Reset
5 – Other
*/
