package tinygo_bno08x

import (
	"math"
	"time"

	"machine"
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

	// MaxChannelNumber is the maximum valid channel number
	MaxChannelNumber uint8 = ChannelGyroRotationVector

	// ExecCommandReset is the command to reset the BNO08x sensor
	ExecCommandReset uint8 = 0x1

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

	// ReportIDUncalibratedGyroscope is the report ID for uncalibrated gyroscope
	ReportIDUncalibratedGyroscope uint8 = 0x7

	// ReportIDGameRotationVector is the report ID for game rotation vector
	ReportIDGameRotationVector uint8 = 0x8

	// ReportIDGeomagneticRotationVector is the report ID for geomagnetic rotation vector
	ReportIDGeomagneticRotationVector uint8 = 0x9

	// ReportIDUncalibratedMagneticField is the report ID for uncalibrated magnetic field
	ReportIDUncalibratedMagneticField uint8 = 0x0F

	// ReportIDTapDetector is the report ID for the tap detector.
	ReportIDTapDetector uint8 = 0x10

	// ReportIDStepCounter is the report ID for the step counter.
	ReportIDStepCounter uint8 = 0x11

	// ReportIDSignificantMotion is the report ID for significant motion detection.
	ReportIDSignificantMotion uint8 = 0x12

	// ReportIDStabilityClassifier is the report ID for the stability classifier.
	ReportIDStabilityClassifier uint8 = 0x13

	// ReportIDRawAccelerometer is the report ID for the raw uncalibrated accelerometer data (ADC units). Used for testing
	ReportIDRawAccelerometer uint8 = 0x14

	// ReportIDRawGyroscope is the report ID for the raw uncalibrated gyroscope data (ADC units).
	ReportIDRawGyroscope uint8 = 0x15

	// ReportIDRawMagnetometer is the report ID for the raw magnetic field measurement (ADC units). Direct data from the magnetometer. Used for testing
	ReportIDRawMagnetometer uint8 = 0x16

	// ReportIDSAR is the report ID for SAR (Specific Absorption Rate) data.
	ReportIDSAR uint8 = 0x17

	// ReportIDStepDetector is the report ID for the step detector.
	ReportIDStepDetector uint8 = 0x18

	// ReportIDShakeDetector is the report ID for the shake detector.
	ReportIDShakeDetector uint8 = 0x19

	// ReportIDFlipDetector is the report ID for the flip detector.
	ReportIDFlipDetector uint8 = 0x1A

	// ReportIDPickupDetector is the report ID for the pickup detector.
	ReportIDPickupDetector uint8 = 0x1B

	// ReportIDStabilityDetector is the report ID for the stability detector.
	ReportIDStabilityDetector uint8 = 0x1C

	// ReportIDActivityClassifier is the report ID for the activity classifier.
	ReportIDActivityClassifier uint8 = 0x1E

	// ReportIDSleepDetector is the report ID for the sleep detector.
	ReportIDSleepDetector uint8 = 0x1F

	// ReportIDTiltDetector is the report ID for the tilt detector.
	ReportIDTiltDetector uint8 = 0x20

	// ReportIDPocketDetector is the report ID for the pocket detector.
	ReportIDPocketDetector uint8 = 0x21

	// ReportIDCircleDetector is the report ID for the circle detector.
	ReportIDCircleDetector uint8 = 0x22

	// ReportIDARVRStabilizedRotationVector is the report ID for the AR/VR stabilized rotation vector.
	ReportIDARVRStabilizedRotationVector uint8 = 0x28

	// ReportIDARVRStabilizedGameRotationVector is the report ID for the AR/VR stabilized game rotation vector.
	ReportIDARVRStabilizedGameRotationVector uint8 = 0x29

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

	// ReportSetFeatureCommandLength is the length of the Set Feature command report
	ReportSetFeatureCommandLength int = 17

	// ReportGetFeatureResponseLength is the length of the Get Feature response report
	ReportGetFeatureResponseLength int = 17

	// ReportProductIDResponseLength is the length of the Product ID response report
	ReportProductIDResponseLength int = 16

	// ReportCommandResponseLength is the length of the command response report
	ReportCommandResponseLength int = 16

	// ReportBaseTimestampLength is the length of the base timestamp report
	ReportBaseTimestampLength int = 5

	// ReportTimestampRebaseLength is the length of the timestamp rebase report
	ReportTimestampRebaseLength int = 5

	// AdvertisementPacketLength is the length of the advertisement packet
	AdvertisementPacketLength int = 272

	// MaxDataLength is the maximum data length for a packet
	MaxDataLength = AdvertisementPacketLength

	// DefaultReportInterval is the default report interval in microseconds
	DefaultReportInterval uint32 = 50_000

	// DebugReportInterval is the debug report interval in microseconds
	DebugReportInterval uint32 = 1_000_000

	// PacketHeaderLength is the length of the Packet header in bytes
	PacketHeaderLength int = 4

	// EnabledActivities is a bitmask for enabled activities. All activities; 1 bit set for each of 8 activities, + Unknown
	EnabledActivities uint32 = 0x1FF

	// PacketBufferSize is the size of the packet buffer
	PacketBufferSize = 512

	// CommandBufferSize is the size of the command buffer
	CommandBufferSize = 12

	// CommandParametersBufferSize is the size of the command parameters buffer
	CommandParametersBufferSize = 9

	// ResetPinDelay is the delay after toggling the reset pin
	ResetPinDelay = 10 * time.Millisecond

	// ResetAttempts is the number of attempts to reset the sensor
	ResetAttempts = 3

	// ResetCommandDelay is the delay after sending a reset command before reading packets
	ResetCommandDelay = 500 * time.Millisecond

	// EnableFeatureAttempts is the number of attempts to enable a feature
	EnableFeatureAttempts = 5

	// CheckIDDelay is the delay after checking the sensor ID
	CheckIDDelay = 500 * time.Millisecond

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

	// UARTRVCBaudRate is the baud rate for UART-RVC communication
	UARTRVCBaudRate uint32 = 115_200 // 115200 for UART-RVC

	// UARTBaudRate is the baud rate for UART communication
	UARTBaudRate uint32 = 3_000_000 // 3Mbps for UART-SHTP

	// UARTDataBits is the number of data bits for UART communication
	UARTDataBits = 8

	// UARTParity is the parity setting for UART communication
	UARTParity = machine.ParityNone

	// UARTStopBits is the number of stop bits for UART communication
	UARTStopBits = 1

	// UARTStartAndEndByte is the start byte and end byte for UART communication
	UARTStartAndEndByte = 0x7E

	// UARTSHTPByte is the SHTP byte for UART communication
	UARTSHTPByte = 0x01

	// UARTControlEscape is the control escape byte for UART communication
	UARTControlEscape = 0x7D

	// UARTXORByte is the XOR character for UART communication
	UARTXORByte = 0x20

	// UARTByteTimeout is the timeout for reading a byte from UART communication
	UARTByteTimeout = 50 * time.Millisecond

	// UARTByteDelay is the delay between bytes when writing to UART
	UARTByteDelay = 1 * time.Millisecond

	// UARTRVCStartByte is the start byte for UART-RVC communication
	UARTRVCStartByte = 0xAA

	// UARTRVCHeaderLength is the length of the UART-RVC header in bytes
	UARTRVCHeaderLength = 2

	// UARTRVCPacketLengthBytes is the number of length bytes in a UART-RVC packet
	UARTRVCPacketLengthBytes = 19

	// SPIWireMode is the SPI mode for BNO08x communication (CPOL=1, CPHA=1)
	SPIWireMode = 3

	// SPIFrequency is the SPI bus frequency in Hz
	SPIFrequency = 1_000_000 // 1MHz

	// SPIIntTimeout is the timeout for waiting for the INT pin to go low
	SPIIntTimeout = 3 * time.Millisecond

	// QuaternionXIndex is index for the X component in quaternion
	QuaternionXIndex = 0

	// QuaternionYIndex is index for the Y component in quaternion
	QuaternionYIndex = 1

	// QuaternionZIndex is index for the Z component in quaternion
	QuaternionZIndex = 2

	// QuaternionWIndex is index for the W component in quaternion
	QuaternionWIndex = 3

	// EulerDegreesRollIndex is the index for the roll component in an euler degrees vector
	EulerDegreesRollIndex = 0

	// EulerDegreesRollMinValue is the minimum value for the Euler degrees roll component
	EulerDegreesRollMinValue = -180.0
	
	// EulerDegreesRollMaxValue is the maximum value for the Euler degrees roll component
	EulerDegreesRollMaxValue = 180.0

	// EulerDegreesPitchIndex is the index for the pitch component in an euler degrees vector
	EulerDegreesPitchIndex = 1

	// EulerDegreesPitchMinValue is the minimum value for the Euler degrees pitch component
	EulerDegreesPitchMinValue = -90.0

	// EulerDegreesPitchMaxValue is the maximum value for the Euler degrees pitch component
	EulerDegreesPitchMaxValue = 90.0

	// EulerDegreesYawIndex is the index for the yaw component in an euler degrees vector
	EulerDegreesYawIndex = 2

	// EulerDegreesYawMinValue is the minimum value for the Euler degrees yaw component
	EulerDegreesYawMinValue = -180.0

	// EulerDegreesYawMaxValue is the maximum value for the Euler degrees yaw component
	EulerDegreesYawMaxValue = 180.0

	// ThreeDimensionalXIndex is the index for the X component in a three-dimensional vector
	ThreeDimensionalXIndex = 0

	// ThreeDimensionalYIndex is the index for the Y component in a three-dimensional vector
	ThreeDimensionalYIndex = 1

	// ThreeDimensionalZIndex is the index for the Z component in a three-dimensional vector
	ThreeDimensionalZIndex = 2

	// Gravity is the standard gravity in m/s^2
	Gravity = 9.80665

	// MilligToMeterPerSecondSquared is the conversion factor from millig to meters per second squared
	MilligToMeterPerSecondSquared = Gravity / 1000.0
)

var (
	// ExecCommandResetData is the command data for the reset command
	ExecCommandResetData = []byte{ExecCommandReset}

	// ReportIDProductIDRequestData is the report ID for the report product ID request data.
	ReportIDProductIDRequestData = []byte{ReportIDProductIDRequest, 0}

	// MaxPackets is the default maximum number of packets to read when waiting for a specific packet type
	MaxPackets = 10

	// WaitForPacketTypeTimeout is the default timeout for waiting for a specific packet type in seconds
	WaitForPacketTypeTimeout = 1 * time.Second

	// MaxClearPendingPacketsTimeout is the maximum timeout for clearing pending packets in seconds
	MaxClearPendingPacketsTimeout = 5 * time.Second

	// CalibrationCommandsTimeout is the timeout for calibration commands in seconds
	CalibrationCommandsTimeout = 5 * time.Second

	// WaitForPacketTimeout is the timeout for waiting for a packet
	WaitForPacketTimeout = 1 * time.Second

	// FeatureEnableTimeout is the timeout for enabling features
	FeatureEnableTimeout = 500 * time.Millisecond

	// PacketReadyCheckDelay is the delay between checks for packet readiness
	PacketReadyCheckDelay = 50 * time.Microsecond

	// UARTRVCTimeout is the timeout for UART-RVC reads
	UARTRVCTimeout = 500 * time.Millisecond

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

	// RawReports are the raw SHTPCommandsNames require their counterpart to be enabled
	RawReports = map[uint8]uint8{
		ReportIDRawAccelerometer: ReportIDAccelerometer,
		ReportIDRawGyroscope:     ReportIDGyroscope,
		ReportIDRawMagnetometer:  ReportIDMagnetometer,
	}

	// SensorReportAccelerometer is the sensor report for the accelerometer
	SensorReportAccelerometer = newSensorReport(
		AccelerometerScalar,
		3,
		10,
	)

	// SensorReportGravity is the sensor report for the gravity vector
	SensorReportGravity = newSensorReport(
		AccelerometerScalar,
		3,
		10,
	)

	// SensorReportGyroscope is the sensor report for the gyroscope
	SensorReportGyroscope = newSensorReport(
		GyroscopeScalar,
		3,
		10,
	)

	// SensorReportMagnetometer is the sensor report for the magnetometer
	SensorReportMagnetometer = newSensorReport(
		MagneticScalar,
		3,
		10,
	)

	// SensorReportLinearAcceleration is the sensor report for the linear acceleration
	SensorReportLinearAcceleration = newSensorReport(
		AccelerometerScalar,
		3,
		10,
	)

	// SensorReportRawAccelerometer is the sensor report for the raw accelerometer
	SensorReportRawAccelerometer = newSensorReport(
		1,
		3,
		16,
	)

	// SensorReportRawGyroscope is the sensor report for the raw gyroscope
	SensorReportRawGyroscope = newSensorReport(
		1,
		3,
		16,
	)

	// SensorReportRawMagnetometer is the sensor report for the raw magnetometer
	SensorReportRawMagnetometer = newSensorReport(
		1,
		3,
		16,
	)

	// SensorReportRotationVector is the sensor report for the rotation vector
	SensorReportRotationVector = newSensorReport(
		QuaternionScalar,
		4,
		14,
	)

	// SensorReportGeomagneticRotationVector is the sensor report for the geomagnetic rotation vector
	SensorReportGeomagneticRotationVector = newSensorReport(
		GeomagneticQuaternionScalar,
		4,
		14,
	)

	// SensorReportGameRotationVector is the sensor report for the game rotation vector
	SensorReportGameRotationVector = newSensorReport(
		QuaternionScalar,
		4,
		12,
	)

	// SensorReportStepCounter is the sensor report for the step counter
	SensorReportStepCounter = newSensorReport(
		1,
		1,
		12,
	)

	// SensorReportShakeDetector is the sensor report for the shake detector
	SensorReportShakeDetector = newSensorReport(
		1,
		1,
		6,
	)

	// SensorReportStabilityClassifier is the sensor report for the stability classifier
	SensorReportStabilityClassifier = newSensorReport(
		1,
		1,
		5,
	)

	// SensorReportActivityClassifier is the sensor report for the activity classifier
	SensorReportActivityClassifier = newSensorReport(
		1,
		1,
		16,
	)

	// headerOnlyPacketMessage is the message printed when a header-only packet is received
	headerOnlyPacketMessage = []byte("Header-only packet received; skipping read")
)
