//go:build tinygo && (rp2040 || rp2350)

package tinygo_bno08x

import (
	"math"
	"time"
)

const (
	// ChannelSHTPCommand is the channel for SHTP commands
	ChannelSHTPCommand uint8 = 0x0

	// ChannelExe is the channel for execution commands
	ChannelExe uint8 = 0x1

	// ChannelControl is the channel for control commands
	ChannelControl uint8 = 0x2

	// ChannelInputSensorReports is the channel for input sensor reports
	ChannelInputSensorReports uint8 = 0x3

	// ChannelWakeInputSensorReports is the channel for wake input sensor reports
	ChannelWakeInputSensorReports uint8 = 0x4

	// ChannelGyroRotationVector is the channel for the gyroscope rotation vector
	ChannelGyroRotationVector uint8 = 0x5

	// CommandReset is the command to reset the BNO08x sensor
	CommandReset uint8 = 0x1

	// SaveDynamicCalibrationData is the command to save the dynamic calibration data
	SaveDynamicCalibrationData uint8 = 0x6

	// MagnetometerCalibration is the command for magnetometer calibration
	MagnetometerCalibration uint8 = 0x7

	// MagnetometerCalibrationConfig is the command to configure the calibration settings
	MagnetometerCalibrationConfig uint8 = 0x0

	// MagnetometerGetCalibration is the command to get the calibration data
	MagnetometerGetCalibration uint8 = 0x1

	// ReportIDAccelerometer is the report ID for calibrated acceleration (m/s2)
	ReportIDAccelerometer uint8 = 0x1

	// ReportIDGyroscope is the report ID for calibrated gyroscope (rad/s).
	ReportIDGyroscope uint8 = 0x2

	// ReportIDMagnetometer is the report ID for magnetic field calibrated (in µTesla). The fully calibrated magnetic field measurement
	ReportIDMagnetometer uint8 = 0x3

	// ReportIDLinearAcceleration is the report ID for linear acceleration (m/s2). Acceleration of the device with gravity removed
	ReportIDLinearAcceleration uint8 = 0x4

	// ReportIDRotationVector is the report ID for rotation vector
	ReportIDRotationVector uint8 = 0x5

	// ReportIDGravity is the report ID for gravity vector (m/s2). Vector direction of gravity
	ReportIDGravity uint8 = 0x6

	// ReportIDGameRotationVector is the report ID for game rotation vector
	ReportIDGameRotationVector uint8 = 0x8

	// ReportIDGeomagneticRotationVector is the report ID for geomagnetic rotation vector
	ReportIDGeomagneticRotationVector uint8 = 0x9

	// ReportIDStepCounter is the report ID for the step counter.
	ReportIDStepCounter uint8 = 0x11

	// ReportIDStabilityClassifier is the report ID for the stability classifier.
	ReportIDStabilityClassifier uint8 = 0x13

	// ReportIDRawAccelerometer is the report ID for the raw uncalibrated accelerometer data (ADC units). Used for testing
	ReportIDRawAccelerometer uint8 = 0x14

	// ReportIDRawGyroscope is the report ID for the raw uncalibrated gyroscope data (ADC units).
	ReportIDRawGyroscope uint8 = 0x15

	// ReportIDRawMagnetometer is the report ID for the raw magnetic field measurement (ADC units). Direct data from the magnetometer. Used for testing
	ReportIDRawMagnetometer uint8 = 0x16

	// ReportIDShakeDetector is the report ID for the shake detector.
	ReportIDShakeDetector uint8 = 0x19

	// ReportIDActivityClassifier is the report ID for the activity classifier.
	ReportIDActivityClassifier uint8 = 0x1E

	// ReportIDCommandResponse is the report ID for command responses
	ReportIDCommandResponse uint8 = 0xF1

	// ReportIDCommandRequest is the report ID for command requests
	ReportIDCommandRequest uint8 = 0xF2

	// ReportIDFRSReadResponse is the report ID for the FRS read response.
	ReportIDFRSReadResponse uint8 = 0xF3

	// ReportIDFRSReadRequest is the report ID for the FRS read request.
	ReportIDFRSReadRequest uint8 = 0xF4

	// ReportIDFRSWriteResponse is the report ID for the FRS write response.
	ReportIDFRSWriteResponse uint8 = 0xF5

	// ReportIDFRSWriteData is the report ID for the FRS write data.
	ReportIDFRSWriteData uint8 = 0xF6

	// ReportIDFRSWriteRequest is the report ID for the FRS (Feature Request Service) write request.
	ReportIDFRSWriteRequest uint8 = 0xF7

	// ReportIDProductIDResponse is the report ID for the report product ID response.
	ReportIDProductIDResponse uint8 = 0xF8

	// ReportIDProductIDRequest is the report ID for the report product ID request.
	ReportIDProductIDRequest uint8 = 0xF9

	// ReportIDTimestampRebase is the report ID for the timestamp rebase.
	ReportIDTimestampRebase uint8 = 0xFA

	// ReportIDBaseTimestamp is the report ID for the base timestamp.
	ReportIDBaseTimestamp uint8 = 0xFB

	// ReportIDGetFeatureResponse is the report ID for the Get Feature request.
	ReportIDGetFeatureResponse uint8 = 0xFC

	// ReportIDSetFeatureCommand is the command ID for setting a feature.
	ReportIDSetFeatureCommand uint8 = 0xFD

	// ReportIDGetFeatureRequest is the report ID for the Get Feature request.
	ReportIDGetFeatureRequest uint8 = 0xFE

	// ReportIDGyroscopeIntegratedRotationVector is the report ID for the gyroscope integrated rotation vector.
	ReportIDGyroscopeIntegratedRotationVector uint8 = 0x2A

	// DefaultReportInterval is the default report interval in microseconds
	DefaultReportInterval uint32 = 60_000

	// DebugReportInterval is the debug report interval in microseconds
	DebugReportInterval uint32 = 1_000_000

	// QuaternionReadTimeout is the timeout for reading quaternion data in seconds
	QuaternionReadTimeout float32 = 0.500

	// PacketReadTimeout is the timeout for reading packets in seconds
	PacketReadTimeout float32 = 2.000

	// FeatureEnableTimeout is the timeout for enabling features in seconds
	FeatureEnableTimeout = 5.0 * time.Second

	// DefaultTimeout is the default timeout for operations in seconds
	DefaultTimeout = 2.0 * time.Second

	// PacketHeaderLength is the length of the Packet header in bytes
	PacketHeaderLength int = 4

	// EnabledActivities is a bitmask for enabled activities. All activities; 1 bit set for each of 8 activities, + Unknown
	EnabledActivities uint32 = 0x1FF

	// DataBufferSize is the size of the data buffer used for reading and writing
	DataBufferSize = 512

	// CommandBufferSize is the size of the command buffer
	CommandBufferSize = 12

	// ResetPacketDelay is the delay after sending a reset command
	ResetPacketDelay = 100 * time.Millisecond

	// InitializeAttempts is the number of attempts to initialize the sensor
	InitializeAttempts = 3

	// CheckIDDelay is the delay after checking the sensor ID
	CheckIDDelay = 500 * time.Millisecond

	// DefaultWaitForPacketTypeTimeout is the default timeout for waiting for a specific packet type in seconds
	DefaultWaitForPacketTypeTimeout = 5 * time.Second

	// DefaultMaxPackets is the default maximum number of packets to read when waiting for a specific packet type
	DefaultMaxPackets = 25

	// I2CDefaultAddress is the default I2C address for the BNO08x sensor
	I2CDefaultAddress uint16 = 0x4A

	// I2CAlternativeAddress is the alternative I2C address for the BNO08x sensor
	I2CAlternativeAddress uint16 = 0x4B

	// I2CFrequency is the I2C bus frequency in Hz
	I2CFrequency = 400_000 // 400 kHz

	// I2CSetupDelay is the delay after setting up the I2C bus
	I2CSetupDelay = 100 * time.Millisecond

	// I2CProbeDeviceAttempts is the number of attempts to probe the device on the I2C bus
	I2CProbeDeviceAttempts = 5

	// I2CProbeDeviceDelay is the delay between attempts to probe the device on the I2C bus
	I2CProbeDeviceDelay = 50 * time.Millisecond

	// UARTBaudRate is the baud rate for UART communication
	UARTBaudRate uint32 = 3_000_000 // 3Mbps for UART-SHTP

	// UARTRVCBaudRate is the baud rate for UART-RVC communication
	UARTRVCBaudRate uint32 = 115_200 // 115200 for UART-RVC

	// UARTStartAndEndByte is the start byte and end byte for UART communication
	UARTStartAndEndByte = 0x7E

	// UARTSHTPByte is the SHTP byte for UART communication
	UARTSHTPByte = 0x01

	// UARTControlEscape is the control escape byte for UART communication
	UARTControlEscape = 0x7D

	// UARTByteTimeout is the timeout for reading a byte from UART communication in milliseconds
	UARTByteTimeout = 250 * time.Millisecond

	// UARTRVCStartByte is the start byte for UART-RVC communication
	UARTRVCStartByte = 0xAA

	// UARTRVCHeaderLength is the length of the UART-RVC header in bytes
	UARTRVCHeaderLength = 2

	// UARTRVCPacketLengthBytes is the number of length bytes in a UART-RVC packet
	UARTRVCPacketLengthBytes = 19

	// EulerDegreesRollIndex is the index for the roll component in an euler degrees vector
	EulerDegreesRollIndex = 0

	// EulerDegreesPitchIndex is the index for the pitch component in an euler degrees vector
	EulerDegreesPitchIndex = 1

	// EulerDegreesYawIndex is the index for the yaw component in an euler degrees vector
	EulerDegreesYawIndex = 2

	// ThreeDimensionalXIndex is the index for the X component in a three-dimensional vector
	ThreeDimensionalXIndex = 0

	// ThreeDimensionalYIndex is the index for the Y component in a three-dimensional vector
	ThreeDimensionalYIndex = 1

	// ThreeDimensionalZIndex is the index for the Z component in a three-dimensional vector
	ThreeDimensionalZIndex = 2

	// DebugHeader is the header for debug messages
	DebugHeader = "[BNO08x]"

	// Gravity is the standard gravity in m/s^2
	Gravity = 9.80665

	// MilligToMeterPerSecondSquared is the conversion factor from millig to meters per second squared
	MilligToMeterPerSecondSquared = Gravity / 1000.0
)

var (
	// ReportIDProductIDRequestData is the report ID for the report product ID request data.
	ReportIDProductIDRequestData = []byte{ReportIDProductIDRequest, 0}

	// QuaternionScalar is the scalar for quaternion values
	QuaternionScalar = math.Pow(2, 14*-1)

	// GeomagneticQuaternionScalar is the scalar for geomagnetic quaternion values
	GeomagneticQuaternionScalar = math.Pow(2, 12*-1)

	// GyroscopeScalar is the scalar for gyroscope values
	GyroscopeScalar = math.Pow(2, 9*-1)

	// AccelerometerScalar is the scalar for accelerometer values
	AccelerometerScalar = math.Pow(2, 8*-1)

	// MagneticScalar is the scalar for magnetic field values
	MagneticScalar = math.Pow(2, 4*-1)

	// ReportLengths is a map of report IDs to their lengths
	ReportLengths = map[uint8]int{
		ReportIDProductIDResponse:  16,
		ReportIDGetFeatureResponse: 17,
		ReportIDCommandResponse:    16,
		ReportIDBaseTimestamp:      5,
		ReportIDTimestampRebase:    5,
	}

	// RawReports are the raw SHTPCommandsNames require their counterpart to be enabled
	RawReports = map[uint8]uint8{
		ReportIDRawAccelerometer: ReportIDAccelerometer,
		ReportIDRawGyroscope:     ReportIDGyroscope,
		ReportIDRawMagnetometer:  ReportIDMagnetometer,
	}

	// SensorReportAccelerometer is the sensor report for the accelerometer
	SensorReportAccelerometer = &sensorReport{
		AccelerometerScalar,
		3,
		10,
	}

	// SensorReportGravity is the sensor report for the gravity vector
	SensorReportGravity = &sensorReport{
		AccelerometerScalar,
		3,
		10,
	}

	// SensorReportGyroscope is the sensor report for the gyroscope
	SensorReportGyroscope = &sensorReport{
		GyroscopeScalar,
		3,
		10,
	}

	// SensorReportMagnetometer is the sensor report for the magnetometer
	SensorReportMagnetometer = &sensorReport{
		MagneticScalar,
		3,
		10,
	}

	// SensorReportLinearAcceleration is the sensor report for the linear acceleration
	SensorReportLinearAcceleration = &sensorReport{
		AccelerometerScalar,
		3,
		10,
	}

	// SensorReportRawAccelerometer is the sensor report for the raw accelerometer
	SensorReportRawAccelerometer = &sensorReport{
		1,
		3,
		16,
	}

	// SensorReportRawGyroscope is the sensor report for the raw gyroscope
	SensorReportRawGyroscope = &sensorReport{
		1,
		3,
		16,
	}

	// SensorReportRawMagnetometer is the sensor report for the raw magnetometer
	SensorReportRawMagnetometer = &sensorReport{
		1,
		3,
		16,
	}

	// InitialBnoSensorReportThreeDimensional is the initial state of the BNO sensor report for 3D data
	InitialBnoSensorReportThreeDimensional = [3]float64{0.0, 0.0, 0.0}

	// SensorReportRotationVector is the sensor report for the rotation vector
	SensorReportRotationVector = &sensorReport{
		QuaternionScalar,
		4,
		14,
	}

	// SensorReportGeomagneticRotationVector is the sensor report for the geomagnetic rotation vector
	SensorReportGeomagneticRotationVector = &sensorReport{
		GeomagneticQuaternionScalar,
		4,
		14,
	}

	// SensorReportGameRotationVector is the sensor report for the game rotation vector
	SensorReportGameRotationVector = &sensorReport{
		QuaternionScalar,
		4,
		12,
	}

	// InitialBnoSensorReportFourDimensional is the initial state of the BNO sensor report for 4D data
	InitialBnoSensorReportFourDimensional = [4]float64{0.0, 0.0, 0.0, 0.0}

	// SensorReportStepCounter is the sensor report for the step counter
	SensorReportStepCounter = &sensorReport{
		1,
		1,
		12,
	}

	// InitialBnoStepCount is the initial state of the BNO step count
	InitialBnoStepCount uint16 = 0

	// SensorReportShakeDetector is the sensor report for the shake detector
	SensorReportShakeDetector = &sensorReport{
		1,
		1,
		6,
	}

	// InitialBnoShakeDetected is the initial state of the BNO shake detection
	InitialBnoShakeDetected = false

	// SensorReportStabilityClassifier is the sensor report for the stability classifier
	SensorReportStabilityClassifier = &sensorReport{
		1,
		1,
		6,
	}

	// InitialBnoStabilityClassification is the initial state of the BNO stability classification
	InitialBnoStabilityClassification = "Unknown"

	// SensorReportActivityClassifier is the sensor report for the activity classifier
	SensorReportActivityClassifier = &sensorReport{
		1,
		1,
		16,
	}

	// InitialBnoMostLikelyClassification is the initial state of the BNO most likely classification
	InitialBnoMostLikelyClassification = "Unknown"

	// InitialBnoClassifications is the initial state of the BNO classifications
	InitialBnoClassifications = map[string]int{
		"Tilting":    -1,
		"OnStairs":   -1,
		"On-Foot":    -1,
		"Other":      -1,
		"On-Bicycle": -1,
		"Still":      -1,
		"Walking":    -1,
		"Unknown":    -1,
		"Running":    -1,
		"In-Vehicle": -1,
	}

	// AvailableSensorReports is a map of available sensor reports
	AvailableSensorReports = map[uint8]*sensorReport{
		ReportIDAccelerometer:             SensorReportAccelerometer,
		ReportIDGravity:                   SensorReportGravity,
		ReportIDGyroscope:                 SensorReportGyroscope,
		ReportIDMagnetometer:              SensorReportMagnetometer,
		ReportIDLinearAcceleration:        SensorReportLinearAcceleration,
		ReportIDRotationVector:            SensorReportRotationVector,
		ReportIDGeomagneticRotationVector: SensorReportGeomagneticRotationVector,
		ReportIDGameRotationVector:        SensorReportGameRotationVector,
		ReportIDStepCounter:               SensorReportStepCounter,
		ReportIDShakeDetector:             SensorReportShakeDetector,
		ReportIDStabilityClassifier:       SensorReportStabilityClassifier,
		ReportIDActivityClassifier:        SensorReportActivityClassifier,
		ReportIDRawAccelerometer:          SensorReportRawAccelerometer,
		ReportIDRawGyroscope:              SensorReportRawGyroscope,
		ReportIDRawMagnetometer:           SensorReportRawMagnetometer,
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
