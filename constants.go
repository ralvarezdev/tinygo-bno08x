package go_adafruit_bno055

import (
	"math"
)

const (
	// Channel 0: the SHTP command channel
	BnoChannelSHTPCommand            int = 0
	BnoChannelExe                    int = 1
	BnoChannelCONTROL                int = 2
	BnoChannelInputSensorReports     int = 3
	BnoChannelWakeInputSensorReports int = 4
	BnoChannelGyroRotationVector     int = 5

	GetFeatureRequest uint8 = 0xFE
	SetFeatureCommand uint8 = 0xFD
	GetFeatureCommand uint8 = 0xFC
	BaseTimestamp     uint8 = 0xFB

	TimestampRebase uint8 = 0xFA

	SHTPReportProductIdResponse uint8 = 0xF8
	SHTPReportProductIdRequest  uint8 = 0xF9

	FRSWriteRequest  uint8 = 0xF7
	FRSWriteData     uint8 = 0xF6
	FRSWriteResponse uint8 = 0xF5

	FRSReadRequest  uint8 = 0xF4
	FRSReadResponse uint8 = 0xF3

	CommandRequest  uint8 = 0xF2
	CommandResponse uint8 = 0xF1

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
	QPoint14Scalar float64 = math.Pow(2, 14*-1)
	QPoint12Scalar float64 = math.Pow(2, 12*-1)
	// QPoint10Scalar float64 = math.Pow(2, 10 * -1)
	QPoint9Scalar float64 = math.Pow(2, 9*-1)
	QPoint8Scalar float64 = math.Pow(2, 8*-1)
	QPoint4Scalar float64 = math.Pow(2, 4*-1)

	GyroscopeScalar             = QPoint9Scalar
	AccelerometerScalar         = QPoint8Scalar
	QuaternionScalar            = QPoint14Scalar
	GeomagneticQuaternionScalar = QPoint12Scalar
	MagneticScalar              = QPoint4Scalar

	ReportLengths = map[uint8]int{
		SHTPReportProductIdResponse: 16,
		GetFeatureCommand:           17,
		CommandResponse:             16,
		BaseTimestamp:               5,
		TimestampRebase:             5,
	}

	// RawReports are the raw Reports require their counterpart to be enabled
	RawReports = map[uint8]uint8{
		BnoReportRawAccelerometer: BnoReportAccelerometer,
		BnoReportRawGyroscope:     BnoReportGyroscope,
		BnoReportRawMagnetometer:  BnoReportMagnetometer,
	}

	AvailableSensorReports = map[uint8][]float64{
		BnoReportAccelerometer:             {QPoint8Scalar, 3, 10},
		BnoReportGravity:                   {QPoint8Scalar, 3, 10},
		BnoReportGyroscope:                 {QPoint9Scalar, 3, 10},
		BnoReportMagnetometer:              {QPoint4Scalar, 3, 10},
		BnoReportLinearAcceleration:        {QPoint8Scalar, 3, 10},
		BnoReportRotationVector:            {QPoint14Scalar, 4, 14},
		BnoReportGeomagneticRotationVector: {QPoint12Scalar, 4, 14},
		BnoReportGameRotationVector:        {QPoint14Scalar, 4, 12},
		BnoReportStepCounter:               {1, 1, 12},
		BnoReportShakeDetector:             {1, 1, 6},
		BnoReportStabilityClassifier:       {1, 1, 6},
		BnoReportActivityClassifier:        {1, 1, 16},
		BnoReportRawAccelerometer:          {1, 3, 16},
		BnoReportRawGyroscope:              {1, 3, 16},
		BnoReportRawMagnetometer:           {1, 3, 16},
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
	DataBufferSize int = 512

	ReportAccuracyStatus = []string{
		"Accuracy Unreliable",
		"Low Accuracy",
		"Medium Accuracy",
		"High Accuracy",
	}
)
