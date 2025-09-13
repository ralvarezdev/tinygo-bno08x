//go:build tinygo && (rp2040 || rp2350)

package tinygo_bno08x

import (
	"encoding/binary"

	tinygotypes "github.com/ralvarezdev/tinygo-types"
	tinygologger "github.com/ralvarezdev/tinygo-logger"
)

type (
	// report represents a BNO08x report
	report struct {
		ID   uint8
		Data []byte
	}

	// sensorReport represents a report from the BNO08x sensor
	sensorReport struct {
		Scalar       float64
		Count        int
		ReportLength int
	}

	// sensorReportData represents a parsed sensor report with 16-bit fields
	sensorReportData struct {
		Count    int
		Results  []float64
		Accuracy ReportAccuracyStatus
	}

	// threeDimensionalReport represents a 3D sensor report
	threeDimensionalReport struct {
		Accuracy ReportAccuracyStatus
		Results  [3]float64
	}

	// FourDimensionalReport represents a 4D sensor report
	fourDimensionalReport struct {
		Accuracy ReportAccuracyStatus
		Results  [4]float64
	}

	// getFeatureReport represents the response report for a Get Feature request
	getFeatureReport struct {
		ReportID                 byte
		FeatureReportID          byte
		FeatureFlags             byte
		ChangeSensitivity        uint16
		ReportInterval           uint32
		BatchIntervalWord        uint32
		SensorSpecificConfigWord uint32
	}

	// shakeReport represents a shake report from the BNO08x device
	shakeReport struct {
		AreShakesDetected bool
	}

	// stepCounterReport represents a step counter report from the BNO08x device
	stepCounterReport struct {
		Count uint16
	}

	// stabilityClassifierReport represents a stability classification report from the BNO08x device
	stabilityClassifierReport struct {
		StabilityClassification ReportStabilityClassification
	}

	// sensorID represents the identification of a sensor
	sensorID struct {
		SoftwareMajorVersion uint8
		SoftwareMinorVersion uint8
		SoftwarePatchVersion uint16
		SoftwarePartNumber   uint32
		SoftwareBuildNumber  uint32
	}

	// commandResponse represents a command response from the BNO08x device
	commandResponse struct {
		SequenceNumber         byte
		Command                byte
		CommandSequenceNumber  byte
		ResponseSequenceNumber byte
		ResponseValues         []byte
	}

	// activityClassifierReport represents an activity classifier report from the BNO08x device
	activityClassifierReport struct {
		SequenceNumber           byte
		Status                   byte
		Delay                    byte
		PageNumber               byte
		MostLikely               byte
		MostLikelyClassification ReportClassification
		Classifications          [ReportClassificationsNumber]int
	}
)

var (
	// sensorIDReportMessage is the message printed when a Sensor ID report is received
	sensorIDReportMessage = []byte("Sensor ID Report")

	// sensorIDReportPartNumberPrefix is the prefix for the part number in the Sensor ID report
	sensorIDReportPartNumberPrefix = []byte("\t Part Number:")

	// sensorIDReportVersionPrefix is the prefix for the version in the Sensor ID report
	sensorIDReportVersionPrefix = []byte("\t Version:")

	// sensorIDReportBuildPrefix is the prefix for the build number in the Sensor ID report
	sensorIDReportBuildPrefix = []byte("\t Build:")

	// sensorIDSoftwareVersionSeparator is the separator for the software version in the Sensor ID report
	sensorIDSoftwareVersionSeparator = []byte(".")
)

// newSensorReport creates a new sensorReport from the provided report bytes.
//
// Parameters:
//
//	scalar: The scalar value for the report
//	count: The count of the report
//	ReportLength: The length of the report in bytes
//
// Returns:
//
//	A sensorReport object containing the scalar, count, and report length
func newSensorReport(scalar float64, count, ReportLength int) sensorReport {
	return sensorReport{
		Scalar:       scalar,
		Count:        count,
		ReportLength: ReportLength,
	}
}


// newReport creates a new report from the Packet data.
//
// Parameters:
//
//	id: The report ID as an uint8.
//	data: A byte slice containing the report data.
//
// Returns:
//
//	A report object containing the ID and data, or an error if the report length is invalid
func newReport(id uint8, data []byte) (report, tinygotypes.ErrorCode) {
	// Check for nil data
	if data == nil {
		return report{}, ErrorCodeBNO08XNilReportData
	}

	// Validate the report length
	expectedLength, err := ReportLength(id)
	if err != tinygotypes.ErrorCodeNil {
		return report{}, ErrorCodeBNO08XFailedToGetExpectedReportLength
	}
	if expectedLength != len(data) {
		return report{}, ErrorCodeBNO08XInvalidReportLength
	}

	return report{
		ID:   id,
		Data: data,
	}, tinygotypes.ErrorCodeNil
}

// newReportFromPacket creates a new report from the provided Packet.
//
// Parameters:
//
//	Packet: A Packet containing the report data.
//
// Returns:
//
// A report object containing the ID and data, or an error if the Packet is nil or if the report ID cannot be retrieved
func newReportFromPacket(packet Packet) (report, tinygotypes.ErrorCode) {
	// Get the report ID from the Packet
	reportID, err := packet.ReportID()
	if err != tinygotypes.ErrorCodeNil {
		return report{}, ErrorCodeBNO08XFailedToGetReportID
	}

	// Create a new report from the Packet data
	return newReport(reportID, packet.Data)
}

// newSetFeatureCommandReport creates a byte slice of the data to enable a feature on the BNO08X sensor.
//
// Parameters:
//
//	featureID: The ID of the feature to enable.
//	reportInterval: The interval for reporting (default: DefaultReportInterval).
//	sensorSpecificConfig: Sensor-specific configuration bits.
// buffer: A byte slice to hold the report data.
//
// Returns:
//
//	An error if the buffer is too short
func newSetFeatureCommandReport(
	featureID uint8,
	reportInterval uint32,
	sensorSpecificConfig uint32,
	buffer []byte,
) tinygotypes.ErrorCode {
	// Check if the buffer slice is nil
	if buffer == nil {
		return ErrorCodeBNO08XSetFeatureEnableReportDataNilBuffer
	}

	// Check if the buffer is too short
	if len(buffer) < 17 {
		return ErrorCodeBNO08XSetFeatureEnableReportDataBufferTooShort
	}

	// Create the set feature report data
	buffer[0] = ReportIDSetFeatureCommand
	buffer[1] = featureID
	binary.LittleEndian.PutUint32(buffer[5:], reportInterval)
	binary.LittleEndian.PutUint32(buffer[13:], sensorSpecificConfig)
	return tinygotypes.ErrorCodeNil
}

// newGetFeatureReport creates a new getFeatureReport from the provided report.
//
// Parameters:
//
//	report: A report containing the report bytes
//
// Returns:
//
//	A getFeatureReport or an error if the report bytes are too short
func newGetFeatureReport(report report) (
	getFeatureReport,
	tinygotypes.ErrorCode,
) {
	// Check if the report ID is valid for a get feature report
	if report.ID != ReportIDGetFeatureResponse {
		return getFeatureReport{}, ErrorCodeBNO08XInvalidReportIDToParseReport
	}

	// Validate the length of the report bytes
	if len(report.Data) != ReportGetFeatureResponseLength {
		return getFeatureReport{}, ErrorCodeBNO08XInvalidReportLength
	}

	return getFeatureReport{
		ReportID:                 report.Data[0],
		FeatureReportID:          report.Data[1],
		FeatureFlags:             report.Data[2],
		ChangeSensitivity:        binary.LittleEndian.Uint16(report.Data[3:5]),
		ReportInterval:           binary.LittleEndian.Uint32(report.Data[5:9]),
		BatchIntervalWord:        binary.LittleEndian.Uint32(report.Data[9:13]),
		SensorSpecificConfigWord: binary.LittleEndian.Uint32(report.Data[13:17]),
	}, tinygotypes.ErrorCodeNil
}

// newShakeReport creates a new shakeReport from the provided report.
//
// Parameters:
//
//	report: A report containing the report bytes
//
// Returns:
//
//	A shakeReport or an error if the report bytes are too short
func newShakeReport(report report) (shakeReport, tinygotypes.ErrorCode) {
	// Check if the report ID is valid for a shake report
	if report.ID != ReportIDShakeDetector {
		return shakeReport{}, ErrorCodeBNO08XInvalidReportIDToParseReport
	}

	// Validate the length of the report bytes
	if len(report.Data) < SensorReportShakeDetector.ReportLength {
		return shakeReport{}, ErrorCodeBNO08XInvalidReportLength
	}

	return shakeReport{
		AreShakesDetected: binary.LittleEndian.Uint16(report.Data[4:6])&0x111 > 0,
	}, tinygotypes.ErrorCodeNil
}

// newStepCounterReport creates a new stepCounterReport from the provided report.
//
// Parameters:
//
//	report: A report containing the report bytes
//
// Returns:
// 
// A stepCounterReport or an error if the report bytes are too short
func newStepCounterReport(report report) (stepCounterReport, tinygotypes.ErrorCode) {
	// Check if the report ID is valid for a step counter report
	if report.ID != ReportIDStepCounter {
		return stepCounterReport{}, ErrorCodeBNO08XInvalidReportIDToParseReport
	}

	// Validate the length of the report bytes
	if len(report.Data) != SensorReportStepCounter.ReportLength {
		return stepCounterReport{}, ErrorCodeBNO08XInvalidReportLength
	}

	return stepCounterReport{
		Count: binary.LittleEndian.Uint16(report.Data[8:10]),
	}, tinygotypes.ErrorCodeNil
}

// newStabilityClassifierReport creates a new stabilityClassifierReport from the provided report.
//
// Parameters:
//
//	report: A report containing the report bytes
//
// Returns:
//
//	A stabilityClassifierReport or an error if the report bytes are too short
func newStabilityClassifierReport(report report) (
	stabilityClassifierReport,
	tinygotypes.ErrorCode,
) {
	// Check if the report ID is valid for a stability classifier report
	if report.ID != ReportIDStabilityClassifier {
		return stabilityClassifierReport{}, ErrorCodeBNO08XInvalidReportIDToParseReport
	}

	// Validate the length of the report bytes
	if len(report.Data) != SensorReportStabilityClassifier.ReportLength {
		return stabilityClassifierReport{}, ErrorCodeBNO08XInvalidReportLength
	}

	// Check if the classification bitfield is within the valid range
	classificationBitfield := report.Data[4]
	stabilityClassification, err := ReportStabilityClassificationFromUint8(classificationBitfield)
	if err != tinygotypes.ErrorCodeNil {
		return stabilityClassifierReport{}, err
	}

	return stabilityClassifierReport{
		StabilityClassification: stabilityClassification,
	}, tinygotypes.ErrorCodeNil
}

// newSensorID parses the sensor ID from the provided report.
//
// Parameters:
//
//	report: A report containing the report bytes
//
// Returns:
//
//	A sensorID or an error if the report ID is invalid or the report length is incorrect
func newSensorID(report report) (sensorID, tinygotypes.ErrorCode) {
	// Check if the report ID is valid for a SHTP report product ID response
	if report.ID != ReportIDProductIDResponse {
		return sensorID{}, ErrorCodeBNO08XInvalidReportIDToParseReport
	}

	// Validate the length of the buffer
	if len(report.Data) != ReportProductIDResponseLength {
		return sensorID{}, ErrorCodeBNO08XInvalidReportLength
	}

	return sensorID{
		SoftwareMajorVersion: report.Data[2],
		SoftwareMinorVersion: report.Data[3],
		SoftwarePatchVersion: binary.LittleEndian.Uint16(report.Data[12:14]),
		SoftwarePartNumber:   binary.LittleEndian.Uint32(report.Data[4:8]),
		SoftwareBuildNumber:  binary.LittleEndian.Uint32(report.Data[8:12]),
	}, tinygotypes.ErrorCodeNil
}

// Log logs the sensor ID details using the provided Logger.
//
// Parameters:
//
//	logger: The Logger instance for logging debug messages.
func (s *sensorID) Log(logger tinygologger.Logger) {
	// Check if the logger is nil
	if logger == nil {
		return
	}

	// Log the sensor ID details
	logger.AddMessage(sensorIDReportMessage, true)

	// Log the software part number
	logger.AddMessageWithUint32(sensorIDReportPartNumberPrefix, s.SoftwarePartNumber, true, true, false)

	// Log the software version
	logger.AddMessageWithUint8(sensorIDReportVersionPrefix, s.SoftwareMajorVersion, true, false, false)
	logger.AddMessageWithUint8(sensorIDSoftwareVersionSeparator, s.SoftwareMinorVersion, false, false, false)
	logger.AddMessageWithUint16(sensorIDSoftwareVersionSeparator, s.SoftwarePatchVersion, false, true, false)

	// Log the software build number
	logger.AddMessageWithUint32(sensorIDReportBuildPrefix, s.SoftwareBuildNumber, true, true, false)

	// Finalize the log
	logger.Info()
}

// newCommandResponse creates a new commandResponse from the provided report.
//
// Parameters:
//
//	report: A report containing the report bytes
//
// Returns:
//
//	A commandResponse or an error if the report bytes are too short
func newCommandResponse(report report) (commandResponse, tinygotypes.ErrorCode) {
	// Check if the report ID is valid for a command response
	if report.ID != ReportIDCommandResponse {
		return commandResponse{}, ErrorCodeBNO08XInvalidReportIDToParseReport
	}

	// Validate the length of the report bytes
	if len(report.Data) != ReportCommandResponseLength {
		return commandResponse{}, ErrorCodeBNO08XInvalidReportLength
	}

	return commandResponse{
		SequenceNumber:         report.Data[1],
		Command:                report.Data[2],
		CommandSequenceNumber:  report.Data[3],
		ResponseSequenceNumber: report.Data[4],
		ResponseValues:         report.Data[5:16],
	}, tinygotypes.ErrorCodeNil
}

// Status is the status of the command response.
//
// Returns:
//
// The status of the command response as a byte
func (cr *commandResponse) Status() byte {
	if len(cr.ResponseValues) < 1 {
		return 0
	}
	return cr.ResponseValues[0]
}

// newActivityClassifierReport creates a new activityClassifierReport from the provided report.
//
// Parameters:
//
//	report: A report containing the report bytes
//
// Returns:
//
//	A activityClassifierReport or an error if the report bytes are too short
func newActivityClassifierReport(report report) (
	activityClassifierReport,
	tinygotypes.ErrorCode,
) {
	// Check if the report ID is valid for an activity classifier report
	if report.ID != ReportIDActivityClassifier {
		return activityClassifierReport{}, ErrorCodeBNO08XInvalidReportIDToParseReport
	}

	// Validate the length of the report bytes
	if len(report.Data) != SensorReportActivityClassifier.ReportLength {
		return activityClassifierReport{}, ErrorCodeBNO08XInvalidReportLength
	}

	mostLikely := report.Data[5]
	pageNumber := report.Data[4] & 0x7F
	confidences := report.Data[6:15]

	// Create a map to hold the classifications with their confidence levels
	classifications := [ReportClassificationsNumber]int{}
	for idx, rawConfidence := range confidences {
		confidence := int(10*pageNumber) + int(rawConfidence)
		classification, err := ReportActivityFromUint8(uint8(idx))
		if err != tinygotypes.ErrorCodeNil {
			return activityClassifierReport{}, err
		}
		classifications[classification] = confidence
	}

	return activityClassifierReport{
		SequenceNumber:           report.Data[1],
		Status:                   report.Data[2],
		Delay:                    report.Data[3],
		PageNumber:               pageNumber,
		MostLikely:               mostLikely,
		MostLikelyClassification: ReportClassificationUnknown,
		Classifications:          classifications,
	}, tinygotypes.ErrorCodeNil
}

// newSensorReportData parses sensor reports with only 16-bit fields.
//
// Parameters:
//
//	report: A report containing the report bytes
//
// Returns:
//
// A sensorReportData or an error if the report bytes are too short
func newSensorReportData(report report) (sensorReportData, tinygotypes.ErrorCode) {
	// The data offset is assumed to be 4 bytes for sensor reports
	dataOffset := 4 // may not always be true

	// Validate the length of the report bytes
	if len(report.Data) < dataOffset {
		return sensorReportData{}, ErrorCodeBNO08XSensorReportDataTooShort
	}

	// Get the sensor report for the given report ID
	sensorReport, err := SensorReportFromReportID(report.ID)
	if err != tinygotypes.ErrorCodeNil {
		return sensorReportData{}, err
	}
	scalar := sensorReport.Scalar
	count := sensorReport.Count

	// Check if it's signed or unsigned data
	formatUnsigned := false
	if _, ok := RawReports[report.ID]; ok {
		formatUnsigned = true
	}

	// Get the accuracy and results from the report bytes
	accuracy, err := ReportAccuracyStatusFromUint8((report.Data)[2] & 0b11)
	if err != tinygotypes.ErrorCodeNil {
		return sensorReportData{}, err
	}
	results := make([]float64, 0, count)

	for offsetIdx := 0; offsetIdx < count; offsetIdx++ {
		// Calculate the total offset for the current data point
		totalOffset := dataOffset + (offsetIdx * 2)
		if totalOffset+2 > len(report.Data) {
			return sensorReportData{}, ErrorCodeBNO08XSensorReportDataTooShort
		}

		// Read the raw data from the report bytes
		var rawData float64
		if formatUnsigned {
			rawData = float64(binary.LittleEndian.Uint16(report.Data[totalOffset : totalOffset+2]))
		} else {
			rawData = float64(int16(binary.LittleEndian.Uint16(report.Data[totalOffset : totalOffset+2])))
		}
		scaledData := rawData * scalar
		results = append(results, scaledData)
	}

	return sensorReportData{
		Count:    count,
		Results:  results,
		Accuracy: accuracy,
	}, tinygotypes.ErrorCodeNil
}

// newThreeDimensionalReport creates a new threeDimensionalReport from the provided report.
//
// Parameters:
//
//	report: A report containing the report bytes
//
// Returns:
//
//	A threeDimensionalReport or an error if the report bytes are too short
func newThreeDimensionalReport(
	report report,
) (threeDimensionalReport, tinygotypes.ErrorCode) {
	// Initialize the sensorReportData
	sensorReportData, err := newSensorReportData(report)
	if err != tinygotypes.ErrorCodeNil {
		return threeDimensionalReport{}, err
	}

	// Ensure the report has exactly 3 results for three-dimensional parsing
	if sensorReportData.Count != 3 {
		return threeDimensionalReport{}, ErrorCodeBNO08XInvalidReportIDForThreeDimensionalParsing
	}

	return threeDimensionalReport{
		Accuracy: sensorReportData.Accuracy,
		Results: [3]float64{
			sensorReportData.Results[0],
			sensorReportData.Results[1],
			sensorReportData.Results[2],
		},
	}, tinygotypes.ErrorCodeNil
}

// newFourDimensionalReport creates a new fourDimensionalReport from the provided report.
//
// Parameters:
//
//	report: A report containing the report bytes
//
// Returns
//
//	A fourDimensionalReport or an error if the report bytes are too short
func newFourDimensionalReport(
	report report,
) (fourDimensionalReport, tinygotypes.ErrorCode) {
	// Initialize the sensorReportData
	sensorReportData, err := newSensorReportData(report)
	if err != tinygotypes.ErrorCodeNil {
		return fourDimensionalReport{}, err
	}

	// Ensure the report has exactly 4 results for four-dimensional parsing
	if sensorReportData.Count != 4 {
		return fourDimensionalReport{}, ErrorCodeBNO08XInvalidReportIDForFourDimensionalParsing
	}

	return fourDimensionalReport{
		Accuracy: sensorReportData.Accuracy,
		Results: [4]float64{
			sensorReportData.Results[0],
			sensorReportData.Results[1],
			sensorReportData.Results[2],
			sensorReportData.Results[3],
		},
	}, tinygotypes.ErrorCodeNil
}

// SensorReportFromReportID returns the sensorReport based on the report ID.
//
// Parameters:
//
//	reportID: The ID of the report
//
// Returns:
//
//	The sensorReport corresponding to the report ID, or an error if the report ID is unknown
func SensorReportFromReportID(reportID uint8) (sensorReport, tinygotypes.ErrorCode) {
	switch reportID {
	case ReportIDAccelerometer:
		return SensorReportAccelerometer, tinygotypes.ErrorCodeNil
	case ReportIDGravity:
		return SensorReportGravity, tinygotypes.ErrorCodeNil
	case ReportIDGyroscope:
		return SensorReportGyroscope, tinygotypes.ErrorCodeNil
	case ReportIDMagnetometer:
		return SensorReportMagnetometer, tinygotypes.ErrorCodeNil
	case ReportIDLinearAcceleration:
		return SensorReportLinearAcceleration, tinygotypes.ErrorCodeNil
	case ReportIDRotationVector:
		return SensorReportRotationVector, tinygotypes.ErrorCodeNil
	case ReportIDGeomagneticRotationVector:
		return SensorReportGeomagneticRotationVector, tinygotypes.ErrorCodeNil
	case ReportIDGameRotationVector:
		return SensorReportGameRotationVector, tinygotypes.ErrorCodeNil
	case ReportIDStepCounter:
		return SensorReportStepCounter, tinygotypes.ErrorCodeNil
	case ReportIDShakeDetector:
		return SensorReportShakeDetector, tinygotypes.ErrorCodeNil
	case ReportIDStabilityClassifier:
		return SensorReportStabilityClassifier, tinygotypes.ErrorCodeNil
	case ReportIDActivityClassifier:
		return SensorReportActivityClassifier, tinygotypes.ErrorCodeNil
	case ReportIDRawAccelerometer:
		return SensorReportRawAccelerometer, tinygotypes.ErrorCodeNil
	case ReportIDRawGyroscope:
		return SensorReportRawGyroscope, tinygotypes.ErrorCodeNil
	case ReportIDRawMagnetometer:
		return SensorReportRawMagnetometer, tinygotypes.ErrorCodeNil
	default:
		return sensorReport{}, ErrorCodeBNO08XUnknownReportID
	}
}

// ReportLength returns the length of the report based on the report ID.
//
// Parameters:
//
//	reportID: The ID of the report
//
// Returns:
//
//	The length of the report in bytes, or an error if the report ID is unknown
func ReportLength(reportID uint8) (int, tinygotypes.ErrorCode) {
	if reportID < 0xF0 {
		sensorReport, err := SensorReportFromReportID(reportID)
		if err != tinygotypes.ErrorCodeNil {
			return 0, err
		}
		return sensorReport.ReportLength, tinygotypes.ErrorCodeNil
	}

	switch reportID {
	case ReportIDProductIDResponse:
		return ReportProductIDResponseLength, tinygotypes.ErrorCodeNil
	case ReportIDGetFeatureResponse:
		return ReportGetFeatureResponseLength, tinygotypes.ErrorCodeNil
	case ReportIDCommandResponse:
		return ReportCommandResponseLength, tinygotypes.ErrorCodeNil
	case ReportIDBaseTimestamp:
		return ReportBaseTimestampLength, tinygotypes.ErrorCodeNil
	case ReportIDTimestampRebase:
		return ReportTimestampRebaseLength, tinygotypes.ErrorCodeNil
	default:
		return 0, ErrorCodeBNO08XUnknownReportID
	}
}

// insertCommandRequestReport inserts a command request report into the provided buffer.
//
// Parameters:
//
//	command: The command to be inserted
//	buffer: A byte slice where the command request report will be inserted
//	nextSequenceNumber: The next sequence number for the command request
//	commandParameters: A slice of integers containing the command parameters
//
// Returns:
//
// An error if the command parameters exceed the limit or if the buffer is too short
func insertCommandRequestReport(
	command uint8,
	buffer []byte,
	nextSequenceNumber uint8,
	commandParameters []byte,
) tinygotypes.ErrorCode {
	// Check for nil buffer
	if buffer == nil {
		return ErrorCodeBNO08XCommandRequestReportNilBuffer
	}

	// Validate the number of command parameters and buffer length
	if commandParameters != nil && len(commandParameters) > 9 {
		return ErrorCodeBNO08XInsertCommandRequestReportTooManyArguments
	}
	if len(buffer) < 12 {
		return ErrorCodeBNO08XInsertCommandRequestReportBufferTooShort
	}

	// Initialize the buffer with zeros
	for i := 0; i < 12; i++ {
		buffer[i] = 0
	}

	// Insert the command request report into the buffer
	buffer[0] = ReportIDCommandRequest
	buffer[1] = byte(nextSequenceNumber)
	buffer[2] = command
	if commandParameters != nil {
		for idx, param := range commandParameters {
			buffer[3+idx] = param
		}
	}
	return tinygotypes.ErrorCodeNil
}
