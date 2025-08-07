package go_adafruit_bno055

import (
	"encoding/binary"
)

type (
	// SensorReport represents a report from the BNO08x sensor
	SensorReport struct {
		Scalar       int
		Count        int
		ReportLength int
	}

	// PacketHeader represents the header of a BNO08x packet
	PacketHeader struct {
		ChannelNumber   uint8
		SequenceNumber  uint8
		DataLength      int
		PacketByteCount int
	}

	// SensorReportData represents a parsed sensor report with 16-bit fields
	SensorReportData struct {
		Results  []int
		Accuracy int
	}

	// GetFeatureResponseReport represents the response report for a Get Feature request
	GetFeatureResponseReport struct {
		ReportID                 byte
		FeatureReportID          byte
		FeatureFlags             byte
		ChangeSensitivity        uint16
		ReportInterval           uint32
		BatchIntervalWord        uint32
		SensorSpecificConfigWord uint32
	}

	// ShakeReport represents a shake report from the BNO08x device
	ShakeReport struct {
		AreShakesDetected bool
	}

	// StepCounterReport represents a step counter report from the BNO08x device
	StepCounterReport struct {
		Count uint16
	}

	// StabilityClassifierReport represents a stability classifier report from the BNO08x device
	StabilityClassifierReport struct {
		StabilityClassifier string
	}

	// SensorID represents the identification of a sensor
	SensorID struct {
		SoftwareMajorVersion uint32
		SoftwareMinorVersion uint32
		SoftwarePatchVersion uint32
		SoftwarePartNumber   uint32
		SoftwareBuildNumber  uint32
	}

	// CommandResponse represents a command response from the BNO08x device
	CommandResponse struct {
		SequenceNumber         byte
		Command                byte
		CommandSequenceNumber  byte
		ResponseSequenceNumber byte
		ResponseValues         []byte
	}

	// ActivityClassifierReport represents an activity classifier report from the BNO08x device
	ActivityClassifierReport struct {
		SequenceNumber           byte
		Status                   byte
		Delay                    byte
		PageNumber               byte
		MostLikely               byte
		MostLikelyClassification string
		Classifications          map[string]int
	}
)

// NewSensorReport creates a new SensorReport from the provided report bytes.
//
// Parameters:
//
//	scalar: The scalar value for the report
//	count: The count of the report
//	reportLength: The length of the report in bytes
//
// Returns:
//
//	A pointer to the newly created SensorReport
func NewSensorReport(scalar, count, reportLength int) *SensorReport {
	return &SensorReport{
		Scalar:       scalar,
		Count:        count,
		ReportLength: reportLength,
	}
}

// NewGetFeatureResponseReport creates a new GetFeatureResponseReport from the provided report bytes.
//
// Parameters:
//
//	reportBytes: The byte slice containing the report data
//
// Returns:
//
//	A pointer to the newly created GetFeatureResponseReport
func NewGetFeatureResponseReport(reportBytes []byte) (
	*GetFeatureResponseReport,
	error,
) {
	// Validate the length of the report bytes
	if len(reportBytes) < 19 {
		return nil, ErrReportBytesTooShort
	}

	report := &GetFeatureResponseReport{
		ReportID:                 reportBytes[0],
		FeatureReportID:          reportBytes[1],
		FeatureFlags:             reportBytes[2],
		ChangeSensitivity:        binary.LittleEndian.Uint16(reportBytes[3:5]),
		ReportInterval:           binary.LittleEndian.Uint32(reportBytes[5:9]),
		BatchIntervalWord:        binary.LittleEndian.Uint32(reportBytes[9:13]),
		SensorSpecificConfigWord: binary.LittleEndian.Uint32(reportBytes[13:17]),
	}

	return report, nil
}

// NewShakeReport creates a new ShakeReport from the provided report bytes.
//
// Parameters:
//
//	reportBytes: The byte slice containing the report data
//
// Returns:
//
//	A pointer to the newly created ShakeReport or an error if the report bytes are too short
func NewShakeReport(reportBytes []byte) (*ShakeReport, error) {
	// Validate the length of the report bytes
	if len(reportBytes) < 6 {
		return nil, ErrReportBytesTooShort
	}

	report := &ShakeReport{
		AreShakesDetected: binary.LittleEndian.Uint16(reportBytes[4:6])&0x111 > 0,
	}
	return report, nil
}

// NewStepCounterReport creates a new StepCounterReport from the provided report bytes.
//
// Parameters:
//
//	reportBytes: The byte slice containing the report data
//
// Returns:
//
//	A pointer to the newly created StepCounterReport or an error if the report bytes are too short
func NewStepCounterReport(reportBytes []byte) (*StepCounterReport, error) {
	// Validate the length of the report bytes
	if len(reportBytes) < 10 {
		return nil, ErrReportBytesTooShort
	}

	report := &StepCounterReport{
		Count: binary.LittleEndian.Uint16(reportBytes[8:10]),
	}
	return report, nil
}

// NewStabilityClassifierReport creates a new StabilityClassifierReport from the provided report bytes.
//
// Parameters:
//
//	reportBytes: The byte slice containing the report data
//
// Returns:
//
//	A pointer to the newly created StabilityClassifierReport or an error if the report bytes are too short
func NewStabilityClassifierReport(reportBytes []byte) (
	*StabilityClassifierReport,
	error,
) {
	// Validate the length of the report bytes
	if len(reportBytes) < 5 {
		return nil, ErrReportBytesTooShort
	}

	classificationBitfield := reportBytes[4]

	// Check if the classification bitfield is within the valid range
	if int(classificationBitfield) >= len(StabilityClassifications) {
		return nil, ErrStabilityClassifierTooShort
	}
	report := &StabilityClassifierReport{
		StabilityClassifier: StabilityClassifications[classificationBitfield],
	}
	return report, nil
}

// NewSensorID parses the sensor ID from the provided buffer.
//
// Parameters:
//
//	buffer: The byte slice containing the sensor ID data
//
// Returns:
//
//	A pointer to the newly created SensorID or an error if the buffer is too short
func NewSensorID(buffer []byte) (*SensorID, error) {
	// Validate the length of the buffer
	if len(buffer) < 14 {
		return nil, ErrBufferTooShort
	}

	if buffer[0] != SHTPReportProductIDResponseID {
		return nil, ErrInvalidReportIDForSensorID
	}

	sensorID := &SensorID{
		SoftwareMajorVersion: uint32(buffer[2]),
		SoftwareMinorVersion: uint32(buffer[3]),
		SoftwarePatchVersion: uint32(binary.LittleEndian.Uint16(buffer[12:14])),
		SoftwarePartNumber:   binary.LittleEndian.Uint32(buffer[4:8]),
		SoftwareBuildNumber:  binary.LittleEndian.Uint32(buffer[8:12]),
	}
	return sensorID, nil
}

// NewCommandResponse creates a new CommandResponse from the provided report bytes.
//
// Parameters:
//
//	reportBytes: The byte slice containing the command response data
//
// Returns:
//
//	A pointer to the newly created CommandResponse or an error if the report bytes are too short
func NewCommandResponse(reportBytes []byte) (*CommandResponse, error) {
	// Validate the length of the report bytes
	if len(reportBytes) < 16 {
		return nil, ErrReportBytesTooShort
	}

	if reportBytes[0] != CommandResponseID {
		return nil, ErrInvalidReportIDForCommandResponse
	}

	commandResponse := &CommandResponse{
		SequenceNumber:         reportBytes[1],
		Command:                reportBytes[2],
		CommandSequenceNumber:  reportBytes[3],
		ResponseSequenceNumber: reportBytes[4],
		ResponseValues:         reportBytes[5:16],
	}
	return commandResponse, nil
}

// NewActivityClassifierReport creates a new ActivityClassifierReport from the provided report bytes.
//
// Parameters:
//
//	reportBytes: The byte slice containing the activity classifier report data
//
// Returns:
//
//	A pointer to the newly created ActivityClassifierReport or an error if the report bytes are too short
func NewActivityClassifierReport(reportBytes []byte) (
	*ActivityClassifierReport,
	error,
) {
	// Validate the length of the report bytes
	if len(reportBytes) < 15 {
		return nil, ErrReportBytesTooShort
	}

	if reportBytes[0] != BnoReportActivityClassifier {
		return nil, ErrInvalidReportIDForActivityClassifier
	}

	mostLikely := reportBytes[5]
	pageNumber := reportBytes[4] & 0x7F
	confidences := reportBytes[6:15]

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

	report := &ActivityClassifierReport{
		SequenceNumber:           reportBytes[1],
		Status:                   reportBytes[2],
		Delay:                    reportBytes[3],
		PageNumber:               pageNumber,
		MostLikely:               mostLikely,
		MostLikelyClassification: mostLikelyClassification,
		Classifications:          classifications,
	}
	return report, nil
}

// NewSensorReportData parses sensor reports with only 16-bit fields.
func NewSensorReportData(reportBytes []byte) (*SensorReportData, error) {
	dataOffset := 4 // may not always be true

	// Validate the length of the report bytes
	if len(reportBytes) < dataOffset {
		return nil, ErrReportBytesTooShort
	}

	// Check if the report ID is valid
	reportID := reportBytes[0]
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
	accuracy := int(reportBytes[2] & 0b11)
	results := make([]int, 0, count)

	for offsetIdx := 0; offsetIdx < count; offsetIdx++ {
		// Calculate the total offset for the current data point
		totalOffset := dataOffset + (offsetIdx * 2)
		if totalOffset+2 > len(reportBytes) {
			return nil, ErrReportBytesTooShort
		}

		// Read the raw data from the report bytes
		var rawData int
		if formatUnsigned {
			rawData = int(binary.LittleEndian.Uint16(reportBytes[totalOffset : totalOffset+2]))
		} else {
			rawData = int(int16(binary.LittleEndian.Uint16(reportBytes[totalOffset : totalOffset+2])))
		}
		scaledData := rawData * scalar
		results = append(results, scaledData)
	}

	report := &SensorReportData{
		Results:  results,
		Accuracy: accuracy,
	}

	return report, nil
}
