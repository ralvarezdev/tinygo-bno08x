package go_adafruit_bno08x

import (
	"encoding/binary"
	"fmt"
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
		StabilityClassification string
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

	// ReportAccuracyStatus is an enumeration of accuracy status values
	ReportAccuracyStatus int
)

const (
	ReportAccuracyStatusUnreliable ReportAccuracyStatus = iota
	ReportAccuracyStatusLow
	ReportAccuracyStatusMedium
	ReportAccuracyStatusHigh
)

// newReport creates a new report from the Packet data.
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

// newReportFromPacket creates a new report from the provided Packet.
//
// Parameters:
//
//	Packet: A pointer to a Packet containing the report data.
//
// Returns:
//
// A pointer to the newly created report or an error if the Packet is nil
func newReportFromPacket(packet *Packet) (*report, error) {
	// Check if the provided Packet is nil
	if packet == nil {
		return nil, ErrNilPacket
	}

	// Get the report ID from the Packet
	reportID, err := packet.ReportID()
	if err != nil {
		return nil, fmt.Errorf("failed to get report ID: %w", err)
	}

	// Create a new report from the Packet data
	return newReport(reportID, &packet.Data)
}

// newReportFromPacketBytes creates a new report from the provided Packet bytes.
//
// Parameters:
//
//	packetBytes: A pointer to a byte slice containing the Packet bytes.
//
// Returns:
//
// A pointer to the newly created report or an error if the Packet bytes are nil
func newReportFromPacketBytes(packetBytes *[]byte) (*report, error) {
	// Check if the provided Packet bytes are nil
	if packetBytes == nil {
		return nil, ErrNilPacketBytes
	}

	// Create a new Packet from the Packet bytes
	packet, err := NewPacket(packetBytes)
	if err != nil {
		return nil, fmt.Errorf("failed to create Packet from bytes: %w", err)
	}

	// Create a new report from the Packet
	return newReportFromPacket(packet)
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
func newSensorReport(scalar float64, count, reportLength int) *sensorReport {
	return &sensorReport{
		Scalar:       scalar,
		Count:        count,
		ReportLength: reportLength,
	}
}

// newGetFeatureReport creates a new getFeatureReport from the provided report.
//
// Parameters:
//
//	report: A pointer to a report containing the report bytes
//
// Returns:
//
//	A pointer to the newly created getFeatureReport
func newGetFeatureReport(report *report) (
	*getFeatureReport,
	error,
) {
	// Check if the provided report is nil
	if report == nil {
		return nil, ErrNilReport
	}

	// Check if the report ID is valid for a get feature report
	if report.ID != ReportIDGetFeatureResponse {
		return nil, fmt.Errorf(
			ErrInvalidReportIDForReportParsing,
			ReportIDGetFeatureResponse,
			report.ID,
		)
	}

	// Validate the length of the report bytes
	if len(report.Data) < 19 {
		return nil, ErrReportDataTooShort
	}

	return &getFeatureReport{
		ReportID:                 report.Data[0],
		FeatureReportID:          report.Data[1],
		FeatureFlags:             report.Data[2],
		ChangeSensitivity:        binary.LittleEndian.Uint16(report.Data[3:5]),
		ReportInterval:           binary.LittleEndian.Uint32(report.Data[5:9]),
		BatchIntervalWord:        binary.LittleEndian.Uint32(report.Data[9:13]),
		SensorSpecificConfigWord: binary.LittleEndian.Uint32(report.Data[13:17]),
	}, nil
}

// newShakeReport creates a new shakeReport from the provided report.
//
// Parameters:
//
//	report: A pointer to a report containing the report bytes
//
// Returns:
//
//	A pointer to the newly created shakeReport or an error if the report bytes are too short
func newShakeReport(report *report) (*shakeReport, error) {
	// Check if the provided report is nil
	if report == nil {
		return nil, ErrNilReport
	}

	// Check if the report ID is valid for a shake report
	if report.ID != ReportIDShakeDetector {
		return nil, fmt.Errorf(
			ErrInvalidReportIDForReportParsing,
			ReportIDShakeDetector,
			report.ID,
		)
	}

	// Validate the length of the report bytes
	if len(report.Data) < 6 {
		return nil, ErrReportDataTooShort
	}

	return &shakeReport{
		AreShakesDetected: binary.LittleEndian.Uint16(report.Data[4:6])&0x111 > 0,
	}, nil
}

// newStepCounterReport creates a new stepCounterReport from the provided report.
//
// Parameters:
//
//	report: A pointer to a report containing the report bytes
//
// Returns:
//
//	A pointer to the newly created stepCounterReport or an error if the report bytes are too short
func newStepCounterReport(report *report) (*stepCounterReport, error) {
	// Check if the provided report is nil
	if report == nil {
		return nil, ErrNilReport
	}

	// Check if the report ID is valid for a step counter report
	if report.ID != ReportIDStepCounter {
		return nil, fmt.Errorf(
			ErrInvalidReportIDForReportParsing,
			ReportIDStepCounter,
			report.ID,
		)
	}

	// Validate the length of the report bytes
	if len(report.Data) < 10 {
		return nil, ErrReportDataTooShort
	}

	return &stepCounterReport{
		Count: binary.LittleEndian.Uint16(report.Data[8:10]),
	}, nil
}

// newStabilityClassifierReport creates a new stabilityClassifierReport from the provided report.
//
// Parameters:
//
//	report: A pointer to a report containing the report bytes
//
// Returns:
//
//	A pointer to the newly created stabilityClassifierReport or an error if the report bytes are too short
func newStabilityClassifierReport(report *report) (
	*stabilityClassifierReport,
	error,
) {
	// Check if the provided report is nil
	if report == nil {
		return nil, ErrNilReport
	}

	// Check if the report ID is valid for a stability classifier report
	if report.ID != ReportIDStabilityClassifier {
		return nil, fmt.Errorf(
			ErrInvalidReportIDForReportParsing,
			ReportIDStabilityClassifier,
			report.ID,
		)
	}

	// Validate the length of the report bytes
	if len(report.Data) < 5 {
		return nil, ErrReportDataTooShort
	}

	classificationBitfield := report.Data[4]

	// Check if the classification bitfield is within the valid range
	if int(classificationBitfield) >= len(StabilityClassifications) {
		return nil, ErrStabilityClassifierTooShort
	}

	return &stabilityClassifierReport{
		StabilityClassification: StabilityClassifications[classificationBitfield],
	}, nil
}

// newSensorID parses the sensor ID from the provided report.
//
// Parameters:
//
//	report: A pointer to a report containing the report bytes
//
// Returns:
//
//	A pointer to the newly created sensorID or an error if the buffer is too short
func newSensorID(report *report) (*sensorID, error) {
	// Check if the provided report is nil
	if report == nil {
		return nil, ErrNilReport
	}

	// Check if the report ID is valid for a SHTP report product ID response
	if report.ID != ReportIDSHTPReportProductIDResponse {
		return nil, fmt.Errorf(
			ErrInvalidReportIDForReportParsing,
			ReportIDSHTPReportProductIDResponse,
			report.ID,
		)
	}

	// Validate the length of the buffer
	if len(report.Data) < 14 {
		return nil, ErrBufferTooShort
	}

	return &sensorID{
		SoftwareMajorVersion: uint32(report.Data[2]),
		SoftwareMinorVersion: uint32(report.Data[3]),
		SoftwarePatchVersion: uint32(binary.LittleEndian.Uint16(report.Data[12:14])),
		SoftwarePartNumber:   binary.LittleEndian.Uint32(report.Data[4:8]),
		SoftwareBuildNumber:  binary.LittleEndian.Uint32(report.Data[8:12]),
	}, nil
}

// newCommandResponse creates a new commandResponse from the provided report.
//
// Parameters:
//
//	report: A pointer to a report containing the report bytes
//
// Returns:
//
//	A pointer to the newly created commandResponse or an error if the report bytes are too short
func newCommandResponse(report *report) (*commandResponse, error) {
	// Check if the provided report is nil
	if report == nil {
		return nil, ErrNilReport
	}

	// Check if the report ID is valid for a command response
	if report.ID != ReportIDCommandResponse {
		return nil, fmt.Errorf(
			ErrInvalidReportIDForReportParsing,
			ReportIDCommandResponse,
			report.ID,
		)
	}

	// Validate the length of the report bytes
	if len(report.Data) < 16 {
		return nil, ErrReportDataTooShort
	}

	return &commandResponse{
		SequenceNumber:         report.Data[1],
		Command:                report.Data[2],
		CommandSequenceNumber:  report.Data[3],
		ResponseSequenceNumber: report.Data[4],
		ResponseValues:         report.Data[5:16],
	}, nil
}

// Status is the status of the command response.
//
// Returns:
//
// The status of the command response as a byte
func (cr *commandResponse) Status() byte {
	if cr == nil || len(cr.ResponseValues) < 1 {
		return 0
	}
	return cr.ResponseValues[0]
}

// newActivityClassifierReport creates a new activityClassifierReport from the provided report.
//
// Parameters:
//
//	report: A pointer to a report containing the report bytes
//
// Returns:
//
//	A pointer to the newly created activityClassifierReport or an error if the report bytes are too short
func newActivityClassifierReport(report *report) (
	*activityClassifierReport,
	error,
) {
	// Check if the provided report is nil
	if report == nil {
		return nil, ErrNilReport
	}

	// Check if the report ID is valid for an activity classifier report
	if report.ID != ReportIDActivityClassifier {
		return nil, fmt.Errorf(
			ErrInvalidReportIDForReportParsing,
			ReportIDActivityClassifier,
			report.ID,
		)
	}

	// Validate the length of the report bytes
	if len(report.Data) < 15 {
		return nil, ErrReportDataTooShort
	}

	mostLikely := report.Data[5]
	pageNumber := report.Data[4] & 0x7F
	confidences := report.Data[6:15]

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
		SequenceNumber:           report.Data[1],
		Status:                   report.Data[2],
		Delay:                    report.Data[3],
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
//	report: A pointer to a report containing the report bytes
//
// Returns:
//
//	A pointer to the newly created sensorReportData or an error if the report bytes are too short
func newSensorReportData(report *report) (*sensorReportData, error) {
	// Check if the provided report is nil
	if report == nil {
		return nil, ErrNilReport
	}

	// The data offset is assumed to be 4 bytes for sensor reports
	dataOffset := 4 // may not always be true

	// Validate the length of the report bytes
	if len(report.Data) < dataOffset {
		return nil, ErrReportDataTooShort
	}

	// Check if the report ID is valid
	sensorReport, ok := AvailableSensorReports[report.ID]
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
	if _, ok := RawReports[report.ID]; ok {
		formatUnsigned = true
	}

	// Get the accuracy and results from the report bytes
	accuracy := ReportAccuracyStatus((report.Data)[2] & 0b11)
	results := make([]float64, 0, count)

	for offsetIdx := 0; offsetIdx < count; offsetIdx++ {
		// Calculate the total offset for the current data point
		totalOffset := dataOffset + (offsetIdx * 2)
		if totalOffset+2 > len(report.Data) {
			return nil, ErrReportDataTooShort
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

	return &sensorReportData{
		Count:    count,
		Results:  results,
		Accuracy: accuracy,
	}, nil
}

// newThreeDimensionalReport creates a new threeDimensionalReport from the provided report.
//
// Parameters:
//
//	report: A pointer to a report containing the report bytes
//
// Returns:
//
//	A pointer to the newly created threeDimensionalReport or an error if the report bytes are too short
func newThreeDimensionalReport(
	report *report,
) (*threeDimensionalReport, error) {
	// Initialize the sensorReportData
	sensorReportData, err := newSensorReportData(report)
	if err != nil {
		return nil, err
	}

	// Ensure the report has exactly 3 results for three-dimensional parsing
	if sensorReportData.Count != 3 {
		return nil, ErrInvalidReportIDForThreeDimensionalParsing
	}

	return &threeDimensionalReport{
		Accuracy: sensorReportData.Accuracy,
		Results: [3]float64{
			sensorReportData.Results[0],
			sensorReportData.Results[1],
			sensorReportData.Results[2],
		},
	}, nil
}

// newFourDimensionalReport creates a new fourDimensionalReport from the provided report.
//
// Parameters:
//
//	report: A pointer to a report containing the report bytes
//
// Returns
//
//	A pointer to the newly created fourDimensionalReport or an error if the report bytes are too short
func newFourDimensionalReport(
	report *report,
) (*fourDimensionalReport, error) {
	// Initialize the sensorReportData
	sensorReportData, err := newSensorReportData(report)
	if err != nil {
		return nil, err
	}

	// Ensure the report has exactly 4 results for four-dimensional parsing
	if sensorReportData.Count != 4 {
		return nil, ErrInvalidReportIDForFourDimensionalParsing
	}

	return &fourDimensionalReport{
		Accuracy: sensorReportData.Accuracy,
		Results: [4]float64{
			sensorReportData.Results[0],
			sensorReportData.Results[1],
			sensorReportData.Results[2],
			sensorReportData.Results[3],
		},
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
	command uint8,
	buffer *[]byte,
	nextSequenceNumber uint8,
	commandParameters *[]byte,
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
	(*buffer)[0] = ReportIDCommandRequest
	(*buffer)[1] = byte(nextSequenceNumber)
	(*buffer)[2] = command
	if commandParameters == nil {
		return nil
	}
	for idx, param := range *commandParameters {
		(*buffer)[3+idx] = param
	}
	return nil
}
