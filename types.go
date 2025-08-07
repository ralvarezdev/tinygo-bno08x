package go_adafruit_bno055

import "encoding/binary"

type (
	// PacketHeader represents the header of a BNO08x packet
	PacketHeader struct {
		ChannelNumber   uint8
		SequenceNumber  uint8
		DataLength      int
		PacketByteCount int
	}

	// GetFeatureResponseReport represents the response report for a Get Feature request
	GetFeatureResponseReport struct {
		ReportID             byte
		FeatureReportID      byte
		FeatureFlags         byte
		ChangeSensitivity    uint16
		ReportInterval       uint32
		BatchIntervalWord    uint32
		SensorSpecificConfig uint32
	}

	// ShakeReport represents a shake report from the BNO08x device
	ShakeReport struct {
		AreShakesDetected bool
	}

	// StepCounterReport represents a step counter report from the BNO08x device
	StepCounterReport struct {
		Count uint16
	}
)

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
		ReportID:             reportBytes[0],
		FeatureReportID:      reportBytes[1],
		FeatureFlags:         reportBytes[2],
		ChangeSensitivity:    binary.LittleEndian.Uint16(reportBytes[3:5]),
		ReportInterval:       binary.LittleEndian.Uint32(reportBytes[5:9]),
		BatchIntervalWord:    binary.LittleEndian.Uint32(reportBytes[9:13]),
		SensorSpecificConfig: binary.LittleEndian.Uint32(reportBytes[13:17]),
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

	shakeBitfield := binary.LittleEndian.Uint16(reportBytes[4:6])
	areShakesDetected := shakeBitfield&0x111 > 0
	report := &ShakeReport{
		AreShakesDetected: areShakesDetected,
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

	count := binary.LittleEndian.Uint16(reportBytes[8:10])
	report := &StepCounterReport{
		Count: count,
	}
	return report, nil
}
