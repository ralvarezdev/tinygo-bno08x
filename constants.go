package go_adafruit_bno055

import (
	"math"
)

const (
	// Channel 0: the SHTP command channel
	BnoChannelSHTPCommand            uint8 = 0
	BnoChannelExe                    uint8 = 1
	BnoChannelCONTROL                uint8 = 2
	BnoChannelInputSensorReports     uint8 = 3
	BnoChannelWakeInputSensorReports uint8 = 4
	BnoChannelGyroRotationVector     uint8 = 5

	GetFeatureRequestID uint8 = 0xFE
	SetFeatureCommandID uint8 = 0xFD
	GetFeatureCommandID uint8 = 0xFC
	BaseTimestamp       uint8 = 0xFB

	TimestampRebase uint8 = 0xFA

	SHTPReportProductIDResponseID uint8 = 0xF8
	SHTPReportProductIDRequestID  uint8 = 0xF9

	FRSWriteRequestID  uint8 = 0xF7
	FRSWriteDataID     uint8 = 0xF6
	FRSWriteResponseID uint8 = 0xF5

	FRSReadRequestID  uint8 = 0xF4
	FRSReadResponseID uint8 = 0xF3

	CommandRequestID  uint8 = 0xF2
	CommandResponseID uint8 = 0xF1

	// DCD/ ME Calibration commands and sub-commands
	SaveDCD             uint8 = 0x6
	MECalibrate         uint8 = 0x7
	MECalibrationConfig uint8 = 0x00
	MEGetCalibration    uint8 = 0x01

	// BnoReportAccelerometer is for calibrated Acceleration (m/s2)
	BnoReportAccelerometer uint8 = 0x01

	// BnoReportGyroscope is for calibrated gyroscope (rad/s).
	BnoReportGyroscope uint8 = 0x02

	// BnoReportMagnetometer is for magnetic field calibrated (in µTesla). The fully calibrated magnetic field measurement.
	BnoReportMagnetometer uint8 = 0x03

	// BnoReportLinearAcceleration is for linear acceleration (m/s2). Acceleration of the device with gravity removed
	BnoReportLinearAcceleration uint8 = 0x04

	// BnoReportRotationVector is for rotation Vector
	BnoReportRotationVector uint8 = 0x05

	// BnoReportGravity is for gravity Vector (m/s2). Vector direction of gravity
	BnoReportGravity uint8 = 0x06

	// BnoReportGameRotationVector is for Game Rotation Vector
	BnoReportGameRotationVector uint8 = 0x08

	BnoReportGeomagneticRotationVector uint8 = 0x09

	BnoReportStepCounter uint8 = 0x11

	BnoReportRawAccelerometer uint8 = 0x14
	BnoReportRawGyroscope     uint8 = 0x15
	BnoReportRawMagnetometer  uint8 = 0x16
	BnoReportShakeDetector    uint8 = 0x19

	BnoReportStabilityClassifier               uint8 = 0x13
	BnoReportActivityClassifier                uint8 = 0x1E
	BnoReportGyroscopeIntegratedRotationVector uint8 = 0x2A

	DefaultReportInterval float32 = 50000 // in microseconds = 50ms
	QuaternionReadTimeout float32 = 0.500 // timeout in seconds
	PacketReadTimeout     float32 = 2.000 // timeout in seconds
	FeatureEnableTimeout  float32 = 2.0
	DefaultTimeout        float32 = 2.0
	Bno08xCmdReset        uint8   = 0x01
	QuaternionQPoint      int     = 14
	BnoHeaderLen          int     = 4
)

var (
	QuaternionScalar            = int(math.Pow(2, 14*-1))
	GeomagneticQuaternionScalar = int(math.Pow(2, 12*-1))
	GyroscopeScalar             = int(math.Pow(2, 9*-1))
	AccelerometerScalar         = int(math.Pow(2, 8*-1))
	MagneticScalar              = int(math.Pow(2, 4*-1))

	ReportLengths = map[uint8]int{
		SHTPReportProductIDResponseID: 16,
		GetFeatureCommandID:           17,
		CommandResponseID:             16,
		BaseTimestamp:                 5,
		TimestampRebase:               5,
	}

	// RawReports are the raw Reports require their counterpart to be enabled
	RawReports = map[uint8]uint8{
		BnoReportRawAccelerometer: BnoReportAccelerometer,
		BnoReportRawGyroscope:     BnoReportGyroscope,
		BnoReportRawMagnetometer:  BnoReportMagnetometer,
	}

	AvailableSensorReports = map[uint8]*SensorReport{
		BnoReportAccelerometer: NewSensorReport(
			AccelerometerScalar,
			3,
			10,
		),
		BnoReportGravity: NewSensorReport(
			AccelerometerScalar,
			3,
			10,
		),
		BnoReportGyroscope: NewSensorReport(
			GyroscopeScalar,
			3,
			10,
		),
		BnoReportMagnetometer: NewSensorReport(
			MagneticScalar,
			3,
			10,
		),
		BnoReportLinearAcceleration: NewSensorReport(
			AccelerometerScalar,
			3,
			10,
		),
		BnoReportRotationVector: NewSensorReport(
			QuaternionScalar,
			4,
			14,
		),
		BnoReportGeomagneticRotationVector: NewSensorReport(
			GeomagneticQuaternionScalar,
			4,
			14,
		),
		BnoReportGameRotationVector: NewSensorReport(
			QuaternionScalar,
			4,
			12,
		),
		BnoReportStepCounter:         NewSensorReport(1, 1, 12),
		BnoReportShakeDetector:       NewSensorReport(1, 1, 6),
		BnoReportStabilityClassifier: NewSensorReport(1, 1, 6),
		BnoReportActivityClassifier:  NewSensorReport(1, 1, 16),
		BnoReportRawAccelerometer:    NewSensorReport(1, 3, 16),
		BnoReportRawGyroscope:        NewSensorReport(1, 3, 16),
		BnoReportRawMagnetometer:     NewSensorReport(1, 3, 16),
	}

	InitialReports = map[uint8]any{
		BnoReportActivityClassifier: map[string]any{
			"Tilting":     -1,
			"most_likely": "Unknown",
			"OnStairs":    -1,
			"On-Foot":     -1,
			"Other":       -1,
			"On-Bicycle":  -1,
			"Still":       -1,
			"Walking":     -1,
			"Unknown":     -1,
			"Running":     -1,
			"In-Vehicle":  -1,
		},
		BnoReportStabilityClassifier:       "Unknown",
		BnoReportRotationVector:            []int{0.0, 0.0, 0.0, 0.0},
		BnoReportGameRotationVector:        []int{0.0, 0.0, 0.0, 0.0},
		BnoReportGeomagneticRotationVector: []int{0.0, 0.0, 0.0, 0.0},
	}

	EnabledActivities uint = 0x1FF // All activities; 1 bit set for each of 8 activities, + Unknown

	// DataBufferSize obviously eats ram
	DataBufferSize = 512

	// ReportAccuracyStatus is a list of accuracy status strings
	ReportAccuracyStatus = []string{
		"Accuracy Unreliable",
		"Low Accuracy",
		"Medium Accuracy",
		"High Accuracy",
	}

	// Activities is a list of activity strings
	Activities = []string{
		"Unknown",
		"In-Vehicle",
		"On-Bicycle",
		"On-Foot",
		"Still",
		"Tilting",
		"Walking",
		"Running",
		"OnStairs",
	}

	// StabilityClassifications is a list of stability classification strings
	StabilityClassifications = []string{
		"Unknown",
		"On Table",
		"Stationary",
		"Stable",
		"In motion",
	}
)
