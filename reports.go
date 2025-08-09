package go_adafruit_bno055

import (
	"encoding/binary"
)

type (
	// report represents a BNO08x report
	report struct {
		ID   uint8
		Data []byte
	}

	// sensorReport represents a report from the BNO08x sensor
	sensorReport struct {
		Scalar       int
		Count        int
		ReportLength int
	}

	// sensorReportData represents a parsed sensor report with 16-bit fields
	sensorReportData struct {
		Results  []int
		Accuracy int
	}

	// getFeatureResponseReport represents the response report for a Get Feature request
	getFeatureResponseReport struct {
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
		StabilityClassifcation string
	}

	// sensorID represents the identification of a sensor
	sensorID struct {
		SoftwareMajorVersion uint32
		SoftwareMinorVersion uint32
		SoftwarePatchVersion uint32
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
		MostLikelyClassification string
		Classifications          map[string]int
	}
)

// newReport creates a new report from the packet data.
//
// Parameters:
//
//	id: The report ID as an uint8.
//	data: A pointer to a byte slice containing the report data.
//
// Returns:
//
//	A report object containing the ID and data.
func newReport(id uint8, data *[]byte) (*report, error) {
	// Check if the provided data is nil
	if data == nil {
		return nil, ErrNilReportData
	}

	return &report{
		ID:   id,
		Data: *data,
	}, nil
}

// newSensorReport creates a new sensorReport from the provided report bytes.
//
// Parameters:
//
//	scalar: The scalar value for the report
//	count: The count of the report
//	reportLength: The length of the report in bytes
//
// Returns:
//
//	A pointer to the newly created sensorReport
func newSensorReport(scalar, count, reportLength int) *sensorReport {
	return &sensorReport{
		Scalar:       scalar,
		Count:        count,
		ReportLength: reportLength,
	}
}

// newGetFeatureResponseReport creates a new getFeatureResponseReport from the provided report bytes.
//
// Parameters:
//
//	reportBytes: A pointer to a byte slice containing the report data
//
// Returns:
//
//	A pointer to the newly created getFeatureResponseReport
func newGetFeatureResponseReport(reportBytes *[]byte) (
	*getFeatureResponseReport,
	error,
) {
	// Check if the provided reportBytes is nil
	if reportBytes == nil {
		return nil, ErrNilReportBytes
	}

	// Validate the length of the report bytes
	if len(*reportBytes) < 19 {
		return nil, ErrReportBytesTooShort
	}

	return &getFeatureResponseReport{
		ReportID:                 (*reportBytes)[0],
		FeatureReportID:          (*reportBytes)[1],
		FeatureFlags:             (*reportBytes)[2],
		ChangeSensitivity:        binary.LittleEndian.Uint16((*reportBytes)[3:5]),
		ReportInterval:           binary.LittleEndian.Uint32((*reportBytes)[5:9]),
		BatchIntervalWord:        binary.LittleEndian.Uint32((*reportBytes)[9:13]),
		SensorSpecificConfigWord: binary.LittleEndian.Uint32((*reportBytes)[13:17]),
	}, nil
}

// newShakeReport creates a new shakeReport from the provided report bytes.
//
// Parameters:
//
//	reportBytes: A pointer to a byte slice containing the report data
//
// Returns:
//
//	A pointer to the newly created shakeReport or an error if the report bytes are too short
func newShakeReport(reportBytes *[]byte) (*shakeReport, error) {
	// Check if the provided reportBytes is nil
	if reportBytes == nil {
		return nil, ErrNilReportBytes
	}

	// Validate the length of the report bytes
	if len(*reportBytes) < 6 {
		return nil, ErrReportBytesTooShort
	}

	return &shakeReport{
		AreShakesDetected: binary.LittleEndian.Uint16((*reportBytes)[4:6])&0x111 > 0,
	}, nil
}

// newStepCounterReport creates a new stepCounterReport from the provided report bytes.
//
// Parameters:
//
//	reportBytes: A pointer to a byte slice containing the report data
//
// Returns:
//
//	A pointer to the newly created stepCounterReport or an error if the report bytes are too short
func newStepCounterReport(reportBytes *[]byte) (*stepCounterReport, error) {
	// Check if the provided reportBytes is nil
	if reportBytes == nil {
		return nil, ErrNilReportBytes
	}

	// Validate the length of the report bytes
	if len(*reportBytes) < 10 {
		return nil, ErrReportBytesTooShort
	}

	return &stepCounterReport{
		Count: binary.LittleEndian.Uint16((*reportBytes)[8:10]),
	}, nil
}

// newStabilityClassifierReport creates a new stabilityClassifierReport from the provided report bytes.
//
// Parameters:
//
//	reportBytes: A pointer to a byte slice containing the report data
//
// Returns:
//
//	A pointer to the newly created stabilityClassifierReport or an error if the report bytes are too short
func newStabilityClassifierReport(reportBytes *[]byte) (
	*stabilityClassifierReport,
	error,
) {
	// Check if the provided reportBytes is nil
	if reportBytes == nil {
		return nil, ErrNilReportBytes
	}

	// Validate the length of the report bytes
	if len(*reportBytes) < 5 {
		return nil, ErrReportBytesTooShort
	}

	classificationBitfield := (*reportBytes)[4]

	// Check if the classification bitfield is within the valid range
	if int(classificationBitfield) >= len(StabilityClassifications) {
		return nil, ErrStabilityClassifierTooShort
	}

	return &stabilityClassifierReport{
		StabilityClassifcation: StabilityClassifications[classificationBitfield],
	}, nil
}

// newSensorID parses the sensor ID from the provided buffer.
//
// Parameters:
//
//	buffer: A pointer to a byte slice containing the sensor ID data
//
// Returns:
//
//	A pointer to the newly created sensorID or an error if the buffer is too short
func newSensorID(buffer *[]byte) (*sensorID, error) {
	// Check if the provided buffer is nil
	if buffer == nil {
		return nil, ErrNilBuffer
	}

	// Validate the length of the buffer
	if len(*buffer) < 14 {
		return nil, ErrBufferTooShort
	}

	if (*buffer)[0] != SHTPReportProductIDResponseID {
		return nil, ErrInvalidReportIDForSensorID
	}

	return &sensorID{
		SoftwareMajorVersion: uint32((*buffer)[2]),
		SoftwareMinorVersion: uint32((*buffer)[3]),
		SoftwarePatchVersion: uint32(binary.LittleEndian.Uint16((*buffer)[12:14])),
		SoftwarePartNumber:   binary.LittleEndian.Uint32((*buffer)[4:8]),
		SoftwareBuildNumber:  binary.LittleEndian.Uint32((*buffer)[8:12]),
	}, nil
}

// newCommandResponse creates a new commandResponse from the provided report bytes.
//
// Parameters:
//
//	reportBytes: A pointer to a byte slice containing the command response data
//
// Returns:
//
//	A pointer to the newly created commandResponse or an error if the report bytes are too short
func newCommandResponse(reportBytes *[]byte) (*commandResponse, error) {
	// Check if the provided reportBytes is nil
	if reportBytes == nil {
		return nil, ErrNilReportBytes
	}

	// Validate the length of the report bytes
	if len(*reportBytes) < 16 {
		return nil, ErrReportBytesTooShort
	}

	if (*reportBytes)[0] != CommandResponseID {
		return nil, ErrInvalidReportIDForCommandResponse
	}

	return &commandResponse{
		SequenceNumber:         (*reportBytes)[1],
		Command:                (*reportBytes)[2],
		CommandSequenceNumber:  (*reportBytes)[3],
		ResponseSequenceNumber: (*reportBytes)[4],
		ResponseValues:         (*reportBytes)[5:16],
	}, nil
}

// newActivityClassifierReport creates a new activityClassifierReport from the provided report bytes.
//
// Parameters:
//
//	reportBytes: A pointer to a byte slice containing the activity classifier data
//
// Returns:
//
//	A pointer to the newly created activityClassifierReport or an error if the report bytes are too short
func newActivityClassifierReport(reportBytes *[]byte) (
	*activityClassifierReport,
	error,
) {
	// Check if the provided reportBytes is nil
	if reportBytes == nil {
		return nil, ErrNilReportBytes
	}

	// Validate the length of the report bytes
	if len(*reportBytes) < 15 {
		return nil, ErrReportBytesTooShort
	}

	if (*reportBytes)[0] != BnoReportActivityClassifier {
		return nil, ErrInvalidReportIDForActivityClassifier
	}

	mostLikely := (*reportBytes)[5]
	pageNumber := (*reportBytes)[4] & 0x7F
	confidences := (*reportBytes)[6:15]

	// Get the most likely activity classification
	mostLikelyClassification := "Unknown"
	if int(mostLikely) < len(Activities) {
		mostLikelyClassification = Activities[mostLikely]
	}

	// Create a map to hold the classifications with their confidence levels
	classifications := make(map[string]int, len(Activities))
	for idx, rawConfidence := range confidences {
		confidence := int(10*pageNumber) + int(rawConfidence)
		if idx < len(Activities) {
			activityString := Activities[idx]
			classifications[activityString] = confidence
		}
	}

	return &activityClassifierReport{
		SequenceNumber:           (*reportBytes)[1],
		Status:                   (*reportBytes)[2],
		Delay:                    (*reportBytes)[3],
		PageNumber:               pageNumber,
		MostLikely:               mostLikely,
		MostLikelyClassification: mostLikelyClassification,
		Classifications:          classifications,
	}, nil
}

// newSensorReportData parses sensor reports with only 16-bit fields.
//
// Parameters:
//
//	reportBytes: A pointer to a byte slice containing the sensor report data
//
// Returns:
//
//	A pointer to the newly created sensorReportData or an error if the report bytes are too short
func newSensorReportData(reportBytes *[]byte) (*sensorReportData, error) {
	// Check if the provided reportBytes is nil
	if reportBytes == nil {
		return nil, ErrNilReportBytes
	}

	// The data offset is assumed to be 4 bytes for sensor reports
	dataOffset := 4 // may not always be true

	// Validate the length of the report bytes
	if len(*reportBytes) < dataOffset {
		return nil, ErrReportBytesTooShort
	}

	// Check if the report ID is valid
	reportID := (*reportBytes)[0]
	sensorReport, ok := AvailableSensorReports[reportID]
	if sensorReport == nil {
		return nil, ErrNilSensorReport
	}
	if !ok {
		return nil, ErrUnknownReportID
	}
	scalar := sensorReport.Scalar
	count := sensorReport.Count

	// Check if it's signed or unsigned data
	formatUnsigned := false
	if _, ok := RawReports[reportID]; ok {
		formatUnsigned = true
	}

	// Get the accuracy and results from the report bytes
	accuracy := int((*reportBytes)[2] & 0b11)
	results := make([]int, 0, count)

	for offsetIdx := 0; offsetIdx < count; offsetIdx++ {
		// Calculate the total offset for the current data point
		totalOffset := dataOffset + (offsetIdx * 2)
		if totalOffset+2 > len(*reportBytes) {
			return nil, ErrReportBytesTooShort
		}

		// Read the raw data from the report bytes
		var rawData int
		if formatUnsigned {
			rawData = int(binary.LittleEndian.Uint16((*reportBytes)[totalOffset : totalOffset+2]))
		} else {
			rawData = int(int16(binary.LittleEndian.Uint16((*reportBytes)[totalOffset : totalOffset+2])))
		}
		scaledData := rawData * scalar
		results = append(results, scaledData)
	}

	return &sensorReportData{
		Results:  results,
		Accuracy: accuracy,
	}, nil
}

// reportLength returns the length of the report based on the report ID.
//
// Parameters:
//
//	reportID: The ID of the report
//
// Returns:
//
//	The length of the report in bytes
func reportLength(reportID uint8) int {
	if reportID < 0xF0 { // it's a sensor report
		return AvailableSensorReports[reportID].ReportLength
	}

	return ReportLengths[reportID]
}

// isControlReport checks if the report ID is a control report.
//
// Parameters:
//
//	reportID: The ID of the report
//
// Returns:
//
//	true if the report ID is a control report, false otherwise
func isControlReport(reportID uint8) bool {
	return reportID > 0xF0 && reportID < 0xFF
}

// isSensorReport checks if the report ID is a sensor report.
//
// Parameters:
//
//	reportID: The ID of the report
//
// Returns:
//
//	true if the report ID is a sensor report, false otherwise
func isSensorReport(reportID uint8) bool {
	// Check if the report ID is less than 0xF0
	return reportID < 0xF0
}

// insertCommandRequestReport inserts a command request report into the provided buffer.
//
// Parameters:
//
//	command: The command to be inserted
//	buffer: A pointer to a byte slice where the command request report will be inserted
//	nextSequenceNumber: The next sequence number for the command request
//	commandParameters: A pointer to a slice of integers containing the command parameters
//
// Returns:
//
// An error if the command parameters exceed the limit or if the buffer is too short
func insertCommandRequestReport(
	command int,
	buffer *[]byte,
	nextSequenceNumber int,
	commandParameters *[]int,
) error {
	// Check if the provided buffer is nil
	if buffer == nil {
		return ErrNilBuffer
	}

	if commandParameters != nil && len(*commandParameters) > 9 {
		return ErrCommandRequestTooManyArguments
	}
	if len(*buffer) < 12 {
		return ErrBufferTooShort
	}

	// Initialize the buffer with zeros
	for i := 0; i < 12; i++ {
		(*buffer)[i] = 0
	}

	// Insert the command request report into the buffer
	(*buffer)[0] = CommandRequestID
	(*buffer)[1] = byte(nextSequenceNumber)
	(*buffer)[2] = byte(command)
	if commandParameters == nil {
		return nil
	}
	for idx, param := range *commandParameters {
		(*buffer)[3+idx] = byte(param)
	}
	return nil
}
