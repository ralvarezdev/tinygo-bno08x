//go:build tinygo && (rp2040 || rp2350)

package tinygo_bno08x

import (
	tinygotypes "github.com/ralvarezdev/tinygo-types"
	tinygologger "github.com/ralvarezdev/tinygo-logger"
	tinygobuffers "github.com/ralvarezdev/tinygo-buffers"
)

type (
	// PacketHeader represents the header of a BNO08x Packet
	PacketHeader struct {
		ChannelNumber   uint8
		SequenceNumber  uint8
		DataLength      int
		PacketByteCount int
		Buffer          []byte
	}

	// Packet represents a BNO08x Packet
	Packet struct {
		Header PacketHeader
		Data   []byte
	}
)

var (
	// channelSHTPCommandName is the name of the SHTP Command channel
	channelSHTPCommandName = []byte("SHTP_COMMAND")

	// channelExeName is the name of the Exe channel
	channelExeName = []byte("EXE")

	// channelControlName is the name of the Control channel
	channelControlName = []byte("CONTROL")

	// channelInputSensorReportsName is the name of the Input Sensor Reports channel
	channelInputSensorReportsName = []byte("INPUT_SENSOR_REPORTS")

	// channelWakeInputSensorReportsName is the name of the Wake Input Sensor Reports channel
	channelWakeInputSensorReportsName = []byte("WAKE_INPUT_SENSOR_REPORTS")

	// channelGyroRotationVectorName is the name of the Gyro Rotation Vector channel
	channelGyroRotationVectorName = []byte("GYRO_ROTATION_VECTOR")

	// channelUnknownName is the name of an unknown channel
	channelUnknownName = []byte("UNKNOWN_CHANNEL")

	// reportIDAccelerometerName is the name of the Accelerometer report ID
	reportIDAccelerometerName = []byte("ACCELEROMETER")

	// reportIDAVRStabilizedGameRotationVectorName is the name of the AR/VR Stabilized Game Rotation Vector report ID
	reportIDARVRStabilizedGameRotationVectorName = []byte("ARVR_STABILIZED_GAME_ROTATION_VECTOR")

	// reportIDARVRStabilizedRotationVectorName is the name of the AR/VR Stabilized Rotation Vector report ID
	reportIDARVRStabilizedRotationVectorName = []byte("ARVR_STABILIZED_ROTATION_VECTOR")

	// reportIDCircleDetectorName is the name of the Circle Detector report ID
	reportIDCircleDetectorName = []byte("CIRCLE_DETECTOR")

	// reportIDFlipDetectorName is the name of the Flip Detector report ID
	reportIDFlipDetectorName = []byte("FLIP_DETECTOR")

	// reportIDGameRotationVectorName is the name of the Game Rotation Vector report ID
	reportIDGameRotationVectorName = []byte("GAME_ROTATION_VECTOR")

	// reportIDGeomagneticRotationVectorName is the name of the Geomagnetic Rotation Vector report ID
	reportIDGeomagneticRotationVectorName = []byte("GEOMAGNETIC_ROTATION_VECTOR")

	// reportIDGravityName is the name of the Gravity report ID
	reportIDGravityName = []byte("GRAVITY")

	// reportIDGyroscopeName is the name of the Gyroscope report ID
	reportIDGyroscopeName = []byte("GYROSCOPE")

	// reportIDLinearAccelerationName is the name of the Linear Acceleration report ID
	reportIDLinearAccelerationName = []byte("LINEAR_ACCELERATION")

	// reportIDMagnetometerName is the name of the Magnetometer report ID
	reportIDMagnetometerName = []byte("MAGNETIC_FIELD")

	// reportIDActivityClassifierName is the name of the Activity Classifier report ID
	reportIDActivityClassifierName = []byte("PERSONAL_ACTIVITY_CLASSIFIER")

	// reportIDPickupDetectorName is the name of the Pickup Detector report ID
	reportIDPickupDetectorName = []byte("PICKUP_DETECTOR")

	// reportIDPocketDetectorName is the name of the Pocket Detector report ID
	reportIDPocketDetectorName = []byte("POCKET_DETECTOR")

	// reportIDRawAccelerometerName is the name of the Raw Accelerometer report ID
	reportIDRawAccelerometerName = []byte("RAW_ACCELEROMETER")

	// reportIDRawGyroscopeName is the name of the Raw Gyroscope report ID
	reportIDRawGyroscopeName = []byte("RAW_GYROSCOPE")

	// reportIDRawMagnetometerName is the name of the Raw Magnetometer report ID
	reportIDRawMagnetometerName = []byte("RAW_MAGNETOMETER")

	// reportIDRotationVectorName is the name of the Rotation Vector report ID
	reportIDRotationVectorName = []byte("ROTATION_VECTOR")

	// reportIDSARName is the name of the SAR report ID
	reportIDSARName = []byte("SAR")

	// reportIDShakeDetectorName is the name of the Shake Detector report ID
	reportIDShakeDetectorName = []byte("SHAKE_DETECTOR")

	// reportIDSignificantMotionName is the name of the Significant Motion report ID
	reportIDSignificantMotionName = []byte("SIGNIFICANT_MOTION")

	// reportIDSleepDetectorName is the name of the Sleep Detector report ID
	reportIDSleepDetectorName = []byte("SLEEP_DETECTOR")

	// reportIDStabilityClassifierName is the name of the Stability Classifier report ID
	reportIDStabilityClassifierName = []byte("STABILITY_CLASSIFIER")

	// reportIDStabilityDetectorName is the name of the Stability Detector report ID
	reportIDStabilityDetectorName = []byte("STABILITY_DETECTOR")

	// reportIDStepCounterName is the name of the Step Counter report ID
	reportIDStepCounterName = []byte("STEP_COUNTER")

	// reportIDStepDetectorName is the name of the Step Detector report ID
	reportIDStepDetectorName = []byte("STEP_DETECTOR")

	// reportIDTapDetectorName is the name of the Tap Detector report ID
	reportIDTapDetectorName = []byte("TAP_DETECTOR")

	// reportITiltDetectorName is the name of the Tilt Detector report ID
	reportIDTiltDetectorName = []byte("TILT_DETECTOR")

	// reportIDUncalibratedGyroscopeName is the name of the Uncalibrated Gyroscope report ID
	reportIDUncalibratedGyroscopeName = []byte("UNCALIBRATED_GYROSCOPE")

	// reportIDUncalibratedMagneticFieldName is the name of the Uncalibrated Magnetic Field report ID
	reportIDUncalibratedMagneticFieldName = []byte("UNCALIBRATED_MAGNETIC_FIELD")

	// reportIDBaseTimestampName is the name of the Base Timestamp report ID
	reportIDBaseTimestampName = []byte("BASE_TIMESTAMP")

	// reportIDCommandRequestName is the name of the Command Request report ID
	reportIDCommandRequestName = []byte("COMMAND_REQUEST")

	// reportIDCommandResponseName is the name of the Command Response report ID
	reportIDCommandResponseName = []byte("COMMAND_RESPONSE")

	// reportIDFRSReadRequestName is the name of the FRS Read Request report ID
	reportIDFRSReadRequestName = []byte("FRS_READ_REQUEST")

	// reportIDFRSReadResponseName is the name of the FRS Read Response report ID
	reportIDFRSReadResponseName = []byte("FRS_READ_RESPONSE")

	// reportIDFRSWriteDataName is the name of the FRS Write Data report ID
	reportIDFRSWriteDataName = []byte("FRS_WRITE_DATA")

	// reportIDFRSWriteRequestName is the name of the FRS Write Request report ID
	reportIDFRSWriteRequestName = []byte("FRS_WRITE_REQUEST")

	// reportIDFRSWriteResponseName is the name of the FRS Write Response report ID
	reportIDFRSWriteResponseName = []byte("FRS_WRITE_RESPONSE")

	// reportIDGetFeatureRequestName is the name of the Get Feature Request report ID
	reportIDGetFeatureRequestName = []byte("GET_FEATURE_REQUEST")

	// reportIDGetFeatureResponseName is the name of the Get Feature Response report ID
	reportIDGetFeatureResponseName = []byte("GET_FEATURE_RESPONSE")

	// reportIDSetFeatureCommandName is the name of the Set Feature Command report ID
	reportIDSetFeatureCommandName = []byte("SET_FEATURE_COMMAND")

	// reportIDTimestampRebaseName is the name of the Timestamp Rebase report ID
	reportIDTimestampRebaseName = []byte("TIMESTAMP_REBASE")

	// reportIDProductIDRequestName is the name of the Product ID Request report ID
	reportIDProductIDRequestName = []byte("PRODUCT_ID_REQUEST")

	// reportIDProductIDResponseName is the name of the Product ID Response report ID
	reportIDProductIDResponseName = []byte("PRODUCT_ID_RESPONSE")

	// reportIDUnknownName is the name of an unknown report ID
	reportIDUnknownName = []byte("UNKNOWN_COMMAND")

	// ExecCommandResetName is the name of the EXE Command Reset
	ExecCommandResetName = []byte("RESET")

	// ExecCommandUnknownName is the name of an unknown EXE command
	ExecCommandUnknownName = []byte("UNKNOWN_COMMAND")

	// sendingPacketHeaderMessage is the message printed when sending a packet header
	sendingPacketHeaderMessage = []byte("SENDING PACKET HEADER: ")

	// receivedPacketHeaderMessage is the message printed when receiving a packet header
	receivedPacketHeaderMessage = []byte("RECEIVED PACKET HEADER: ")

	// dataLengthPrefix is the prefix for the data length in the packet header log
	dataLengthPrefix = []byte("\t Data Length: ")

	// channelPrefix is the prefix for the channel in the packet header log
	channelPrefix = []byte("\t Channel: ")

	// sequenceNumberPrefix is the prefix for the sequence number in the packet header log
	sequenceNumberPrefix = []byte("\t Sequence number: ")

	// sendingPacketDataMessage is the message printed when sending a packet data
	sendingPacketDataMessage = []byte("SENDING PACKET DATA: ")

	// receivedPacketDataMessage is the message printed when receiving a packet data
	receivedPacketDataMessage = []byte("RECEIVED PACKET DATA: ")

	// noDataMessage is the message printed when there is no data in the packet
	noDataMessage = []byte("\t No data")

	// reportPrefix is the prefix for the report in the packet data log
	reportPrefix = []byte("\t Report: ")

	// reportIDPrefix is the prefix for the report ID in the packet data log
	reportIDPrefix = []byte(" (")

	// reportIDSuffix is the suffix for the report ID in the packet data log
	reportIDSuffix = []byte(")")

	// channelNumberPrefix is the prefix for the channel numberin the packet data log
	channelNumberPrefix = []byte(" (")

	// channelNumberSuffix is the suffix for the channel number in the packet data log
	channelNumberSuffix = []byte(")")

	// sensorReportPrefix is the prefix for the sensor report in the packet data log
	sensorReportPrefix = []byte("\t Sensor Report: ")

	// featureIDPrefix is the prefix for the feature ID in the packet data log
	featureIDPrefix = []byte("\t Feature ID: ")
)

// channelNumberNameBuffer returns the name of the channel as a byte slice.
//
// Parameters:
//
//	channelNumber: The channel number to get the name for.
//
// Returns:
//
// The channel name as a byte slice.
func ChannelNumberNameBuffer(channelNumber uint8) []byte {
	switch channelNumber {
	case ChannelSHTPCommand:
		return channelSHTPCommandName
	case ChannelExe:
		return channelExeName
	case ChannelControl:
		return channelControlName
	case ChannelInputSensorReports:
		return channelInputSensorReportsName
	case ChannelWakeInputSensorReports:
		return channelWakeInputSensorReportsName
	case ChannelGyroRotationVector:
		return channelGyroRotationVectorName
	default:
		return channelUnknownName
	}
}

// SHTPCommandNameBuffer returns the string representation of the SHTP command.
//
// Parameters:
//
// commandID: The command ID to get the name for.
//
// Returns:
//
// The command name as a byte slice or "UNKNOWN_COMMAND" if not found.
func SHTPCommandNameBuffer(commandID uint8) []byte {
	switch commandID {
	case ReportIDAccelerometer:
		return reportIDAccelerometerName
	case ReportIDARVRStabilizedGameRotationVector:
		return reportIDARVRStabilizedGameRotationVectorName
	case ReportIDARVRStabilizedRotationVector:
		return reportIDARVRStabilizedRotationVectorName
	case ReportIDCircleDetector:
		return reportIDCircleDetectorName
	case ReportIDFlipDetector:
		return reportIDFlipDetectorName
	case ReportIDGameRotationVector:
		return reportIDGameRotationVectorName
	case ReportIDGeomagneticRotationVector:
		return reportIDGeomagneticRotationVectorName
	case ReportIDGravity:
		return reportIDGravityName
	case ReportIDGyroscope:
		return reportIDGyroscopeName
	case ReportIDLinearAcceleration:
		return reportIDLinearAccelerationName
	case ReportIDMagnetometer:
		return reportIDMagnetometerName
	case ReportIDActivityClassifier:
		return reportIDActivityClassifierName
	case ReportIDPickupDetector:
		return reportIDPickupDetectorName
	case ReportIDPocketDetector:
		return reportIDPocketDetectorName
	case ReportIDRawAccelerometer:
		return reportIDRawAccelerometerName
	case ReportIDRawGyroscope:
		return reportIDRawGyroscopeName
	case ReportIDRawMagnetometer:
		return reportIDRawMagnetometerName
	case ReportIDRotationVector:
		return reportIDRotationVectorName
	case ReportIDSAR:
		return reportIDSARName
	case ReportIDShakeDetector:
		return reportIDShakeDetectorName
	case ReportIDSignificantMotion:
		return reportIDSignificantMotionName
	case ReportIDSleepDetector:
		return reportIDSleepDetectorName
	case ReportIDStabilityClassifier:
		return reportIDStabilityClassifierName
	case ReportIDStabilityDetector:
		return reportIDStabilityDetectorName
	case ReportIDStepCounter:
		return reportIDStepCounterName
	case ReportIDStepDetector:
		return reportIDStepDetectorName
	case ReportIDTapDetector:
		return reportIDTapDetectorName
	case ReportIDTiltDetector:
		return reportIDTiltDetectorName
	case ReportIDUncalibratedGyroscope:
		return reportIDUncalibratedGyroscopeName
	case ReportIDUncalibratedMagneticField:
		return reportIDUncalibratedMagneticFieldName
	default:
		return reportIDUnknownName
	}
}

// ControlCommandNameBuffer returns the name of the control command as a byte slice.
//
// Parameters:
//
// commandID: The command ID to get the name for.
//
// Returns:
//
// The command name as a byte slice or "UNKNOWN_COMMAND" if not found.
func ControlCommandNameBuffer(commandID uint8) []byte {
	switch commandID {
	case ReportIDBaseTimestamp:
		return reportIDBaseTimestampName
	case ReportIDCommandRequest:
		return reportIDCommandRequestName
	case ReportIDCommandResponse:
		return reportIDCommandResponseName
	case ReportIDFRSReadRequest:
		return reportIDFRSReadRequestName
	case ReportIDFRSReadResponse:
		return reportIDFRSReadResponseName
	case ReportIDFRSWriteData:
		return reportIDFRSWriteDataName
	case ReportIDFRSWriteRequest:
		return reportIDFRSWriteRequestName
	case ReportIDFRSWriteResponse:
		return reportIDFRSWriteResponseName
	case ReportIDGetFeatureRequest:
		return reportIDGetFeatureRequestName
	case ReportIDGetFeatureResponse:
		return reportIDGetFeatureResponseName
	case ReportIDSetFeatureCommand:
		return reportIDSetFeatureCommandName
	case ReportIDTimestampRebase:
		return reportIDTimestampRebaseName
	case ReportIDProductIDRequest:
		return reportIDProductIDRequestName
	case ReportIDProductIDResponse:
		return reportIDProductIDResponseName
	default:
		return reportIDUnknownName
	}
}

// IsControlReportID checks if the report ID is a control report.
//
// Parameters:
//
//	reportID: The ID of the report
//
// Returns:
//
//	true if the report ID is a control report, false otherwise
func IsControlReportID(reportID uint8) bool {
	switch reportID {
	case ReportIDBaseTimestamp,
		ReportIDCommandRequest,
		ReportIDCommandResponse,
		ReportIDFRSReadRequest,
		ReportIDFRSReadResponse,
		ReportIDFRSWriteData,
		ReportIDFRSWriteRequest,
		ReportIDFRSWriteResponse,
		ReportIDGetFeatureRequest,
		ReportIDGetFeatureResponse,
		ReportIDSetFeatureCommand,
		ReportIDTimestampRebase,
		ReportIDProductIDRequest,
		ReportIDProductIDResponse:
		return true
	default:
		return false
	}
}

// ExecCommandNameBuffer returns the name of the EXE command as a byte slice.
//
// Parameters:
//
// commandID: The command ID to get the name for.
//
// Returns:
//
// The command name as a byte slice or "UNKNOWN_COMMAND" if not found.
func ExecCommandNameBuffer(commandID uint8) []byte {
	switch commandID {
	case ExecCommandReset:
		return ExecCommandResetName
	default:
		return ExecCommandUnknownName
	}
}

// NewPacketHeader creates a PacketHeader.
//
// Parameters:
//
// packetByteCount: The total byte count of the Packet.
// channelNumber: The channel number of the Packet.
// sequenceNumber: The sequence number of the Packet.
// buffer: A byte slice to hold the PacketHeader data.
//
// Returns:
//
// A PacketHeader object, or an error if the buffer is too short.
func NewPacketHeader(
	packetByteCount uint16,
	channelNumber uint8,
	sequenceNumber uint8,
	buffer []byte,
) (PacketHeader, tinygotypes.ErrorCode) {
	dataLength := int(packetByteCount) - PacketHeaderLength
	if dataLength < 0 {
		dataLength = 0
	}

	// Check if buffer is nil
	if buffer == nil {
		return PacketHeader{}, ErrorCodeBNO08XNilPacketHeaderBuffer
	}

	// Ensure the buffer is at least PacketHeaderLength bytes long
	if len(buffer) < PacketHeaderLength {
		return PacketHeader{}, ErrorCodeBNO08XReportHeaderBufferTooShort
	}

	// First two bytes are writeLength (little-endian)
	buffer[0] = uint8(packetByteCount & 0xFF)
	buffer[1] = uint8((packetByteCount >> 8) & 0x7F)
	buffer[2] = channelNumber
	buffer[3] = sequenceNumber

	return PacketHeader{
		ChannelNumber:   channelNumber,
		SequenceNumber:  sequenceNumber,
		DataLength:      dataLength,
		PacketByteCount: int(packetByteCount),
		Buffer:          buffer,
	}, tinygotypes.ErrorCodeNil
}

// NewPacketHeaderFromBuffer creates a PacketHeader from a given buffer.
//
// Parameters:
//
//	buffer: A byte slice containing the Packet data.
//
// Returns:
//
//	A PacketHeader object or an error if the buffer is too short.
func NewPacketHeaderFromBuffer(buffer []byte) (PacketHeader, tinygotypes.ErrorCode) {
	// Ensure the buffer is at least PacketHeaderLength bytes long to read the header
	if len(buffer) < PacketHeaderLength {
		return PacketHeader{}, ErrorCodeBNO08XPacketHeaderBufferTooShort
	}

	// Parse the header fields from the buffer
	packetByteCount, _ := tinygobuffers.BytesToUint16LE(buffer[0:2])
	packetByteCount &= 0x7FFF
	channelNumber := buffer[2]
	sequenceNumber := buffer[3]

	return PacketHeader{
		ChannelNumber:   channelNumber,
		SequenceNumber:  sequenceNumber,
		DataLength:      int(packetByteCount) - PacketHeaderLength,
		PacketByteCount: int(packetByteCount),
		Buffer:          buffer,
	}, tinygotypes.ErrorCodeNil
}

// NewPacketHeaderFromData creates a PacketHeader from the provided data.
//
// Parameters:
//
// channelNumber: The channel number of the Packet.
// sequenceNumber: The sequence number of the Packet.
// data: A byte slice containing the Packet data.
// headerBuffer: A byte slice to hold the PacketHeader data.
//
// Returns:
//
// A PacketHeader object or an error if the data is nil.
func NewPacketHeaderFromData(
	channelNumber uint8,
	sequenceNumber uint8,
	data []byte,
	headerBuffer []byte,
) (PacketHeader, tinygotypes.ErrorCode) {
	// Check if data is nil
	if data == nil {
		return PacketHeader{}, ErrorCodeBNO08XNilReportData
	}

	// Calculate packet byte count
	packetByteCount := len(data) + PacketHeaderLength

	// Create PacketHeader
	header, err := NewPacketHeader(
		uint16(packetByteCount),
		channelNumber,
		sequenceNumber,
		headerBuffer,
	)
	if err != tinygotypes.ErrorCodeNil {
		return PacketHeader{}, err
	}

	return header, tinygotypes.ErrorCodeNil
}

// IsError checks if the provided PacketHeader indicates an error condition.
//
// Parameters:
//
//	header: The PacketHeader to check.
//
// Returns:
//
//	True if the header indicates an error, otherwise false.
func (h *PacketHeader) IsError() bool {
	// Check if the channel number is greater than 5
	if h.ChannelNumber > 5 {
		return true
	}
	// Check if the Packet byte count and sequence number indicate an error
	if h.PacketByteCount == 0xFFFF && h.SequenceNumber == 0xFF {
		return true
	}
	return false
}

// Log returns a byte slice representation of the PacketHeader for debugging purposes.
//
// Parameters:
//
// isBeingSent: A boolean indicating if the PacketHeader is being sent (true) or received (false).
// logger: A Logger interface for logging messages.
func (ph *PacketHeader) Log(isBeingSent bool, logger tinygologger.Logger) {
	// Check if logger is nil
	if logger == nil {
		return
	}

	// Title
	if isBeingSent {
		logger.AddMessage(sendingPacketHeaderMessage, true)
	} else {
		logger.AddMessage(receivedPacketHeaderMessage, true)
	}

	// Log the data length
	logger.AddMessageWithUint64(dataLengthPrefix, uint64(ph.DataLength), true, true, false)

	// Log the channel
	logger.AddMessage(channelPrefix, false)
	logger.AddMessage(ChannelNumberNameBuffer(ph.ChannelNumber), false)
	logger.AddMessageWithUint8(channelNumberPrefix, ph.ChannelNumber, true, false, true)
	logger.AddMessage(channelNumberSuffix, true)

	// Log the sequence number
	logger.AddMessageWithUint8(sequenceNumberPrefix, ph.SequenceNumber, true, true, false)
	logger.Debug()
}

// NewPacket creates a new Packet from the provided data and header.
//
// Parameters:
//
//	data: A byte slice containing the Packet data.
//	header: A PacketHeader.
//
// Returns:
//
// A Packet object or an error if the data or header is nil.
func NewPacket(data []byte, header PacketHeader) (Packet, tinygotypes.ErrorCode) {
	// Check if data is nil
	if data == nil {
		return Packet{}, ErrorCodeBNO08XNilReportData
	}

	return Packet{
		Header: header,
		Data:   data,
	}, tinygotypes.ErrorCodeNil
}

// NewPacketFromData creates a new Packet from the provided data.
//
// Parameters:
//
// channelNumber: The channel number of the Packet.
// sequenceNumber: The sequence number of the Packet.
// data: A byte slice containing the Packet data.
// headerBuffer: A byte slice to hold the PacketHeader data.
//
// Returns:
//
// A Packet object or an error if the data is nil.
func NewPacketFromData(
	channelNumber uint8,
	sequenceNumber uint8,
	data []byte,
	headerBuffer []byte,
) (Packet, tinygotypes.ErrorCode) {
	// Check if data is nil
	if data == nil {
		return Packet{}, ErrorCodeBNO08XNilReportData
	}

	// Create PacketHeader from data
	header, err := NewPacketHeaderFromData(
		channelNumber,
		sequenceNumber,
		data,
		headerBuffer,
	)
	if err != tinygotypes.ErrorCodeNil {
		return Packet{}, err
	}

	return Packet{
		Header: header,
		Data:   data,
	}, tinygotypes.ErrorCodeNil
}

// SequenceNumber returns the sequence number of the Packet.
//
// Returns:
//
//	The sequence number as an uint8.
func (p *Packet) SequenceNumber() uint8 {
	return p.Header.SequenceNumber
}

// ReportID returns the report ID of the Packet.
//
// Returns:
//
//	The report ID as an uint8 or an error if the data is too short.
func (p *Packet) ReportID() (uint8, tinygotypes.ErrorCode) {
	if len(p.Data) < 1 {
		return 0, ErrorCodeBNO08XPacketDataTooShort
	}
	return p.Data[0], tinygotypes.ErrorCodeNil
}

// ChannelNumber returns the channel number of the Packet.
//
// Returns:
//
//	The channel number as an uint8.
func (p *Packet) ChannelNumber() uint8 {
	return p.Header.ChannelNumber
}

// PacketByteCount returns the total byte count of the Packet.
//
// Returns:
//
// The total byte count as an int.
func (p *Packet) PacketByteCount() int {
	return p.Header.PacketByteCount
}

// DataLength returns the data length of the Packet.
//
// Returns:
//
// The data length as an int.
func (p *Packet) DataLength() int {
	return p.Header.DataLength
}

// IsError checks if the Packet indicates an error condition.
//
// Returns:
//
//	True if the Packet is an error, otherwise false.
func (p *Packet) IsError() bool {
	return p.Header.IsError()
}

// Log logs the Packet details for debugging purposes.
//
// Parameters:
//
// isBeingSent: A boolean indicating if the Packet is being sent (true) or received (false).
// logHeader: A boolean indicating if the PacketHeader should be logged (true) or not (false).
// logger: A Logger interface for logging messages.
func (p *Packet) Log(isBeingSent bool, logHeader bool, logger tinygologger.Logger) {
	// Check if logger is nil
	if logger == nil {
		return
	}

	// Log header
	if logHeader {
		p.Header.Log(isBeingSent, logger)
	}

	// Title
	if isBeingSent {
		logger.AddMessage(sendingPacketDataMessage, true)
	} else {
		logger.AddMessage(receivedPacketDataMessage, true)
	}

	// Get data length
	dataLength := len(p.Data)

	// Check if there is data
	if dataLength == 0 {
		logger.AddMessage(noDataMessage, true)
		logger.Debug()
		return
	}

	// Get the report ID
	reportID := p.Data[0]

	// Log the report name based on the channel
	logger.AddMessage(reportPrefix, false)
	switch p.ChannelNumber() {
	case ChannelSHTPCommand:
		logger.AddMessage(SHTPCommandNameBuffer(reportID), false)
	case ChannelExe:
		logger.AddMessage(ExecCommandNameBuffer(reportID), false)
	case ChannelControl, ChannelInputSensorReports:
		logger.AddMessage(ControlCommandNameBuffer(reportID), false)
	}

	// Log report ID
	logger.AddMessageWithUint8(reportIDPrefix, reportID, true, false, true)
	logger.AddMessage(reportIDSuffix, true)

	// Additional interpretation (requires at least 6 data bytes)
	if dataLength < 6 {
		logger.Debug()
		return
	}

	// High report IDs (command responses / meta)
	if IsControlReportID(reportID) {
		// Get the sensor report ID
		sensorReportID := p.Data[5]

		// Only log if not unknown
		sensorReportNameBuffer := SHTPCommandNameBuffer(sensorReportID)
		isUnknown := len(sensorReportNameBuffer) == len(reportIDUnknownName)
		if isUnknown {
			isUnknown = true
			for i, _ := range sensorReportNameBuffer {
				if sensorReportNameBuffer[i] != reportIDUnknownName[i] {
					isUnknown = false
					break
				}
			}
		}

		if !isUnknown {
			logger.AddMessage(sensorReportPrefix, false)
			logger.AddMessage(sensorReportNameBuffer, false)
			logger.AddMessageWithUint8(reportIDPrefix, sensorReportID, true, false, true)
			logger.AddMessage(reportIDSuffix, true)
		}
	}

	if reportID == ReportIDGetFeatureResponse || reportID == ReportIDSetFeatureCommand {
		featureID := p.Data[1]
		logger.AddMessageWithUint8(featureIDPrefix, featureID, true, true, true)
	}

	logger.Debug()
	return
}
