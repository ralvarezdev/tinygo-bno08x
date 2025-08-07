package go_adafruit_bno055

import (
	"encoding/binary"
	"errors"
)

// ParseSensorReportData parses a sensor report from the BNO08x device.
func ParseSensorReportData(reportBytes []byte) ([]int, int, error) {
	dataOffset := 4
	reportID := reportBytes[0]
	params := AvailableSensorReports[reportID]
	scalar := int(params[0])
	count := int(params[1])
	var formatUnsigned bool
	if _, ok := RawReports[reportID]; ok {
		formatUnsigned = true
	}
	results := make([]int, 0, count)
	accuracy := int(reportBytes[2] & 0b11)

	for offsetIdx := 0; offsetIdx < count; offsetIdx++ {
		totalOffset := dataOffset + (offsetIdx * 2)
		var rawData int16

		// Check if the report bytes has enough length
		if totalOffset+2 > len(reportBytes) {
			return nil, 0, ErrReportBytesTooShort
		}

		if formatUnsigned {
			rawData = int16(binary.LittleEndian.Uint16(reportBytes[totalOffset : totalOffset+2]))
		} else {
			rawData = int16(binary.LittleEndian.Uint16(reportBytes[totalOffset : totalOffset+2]))
		}
		scaledData := int(rawData) * scalar
		results = append(results, scaledData)
	}

	return results, accuracy
}

////////######## PACKET PARSING ###########################
func _parse_sensor_report_data(report_bytes: bytearray) -> tuple[tuple, int]:
	"""Parses Reports with only 16-bit fields"""
	data_offset = 4  // this may not always be true
	report_id = report_bytes[0]
	scalar, count, _report_length = AvailableSensorReports[report_id]
	if report_id in RawReports:
	// raw Reports are unsigned
		format_str = "<H"
	else:
		format_str = "<h"
	results = []
	accuracy = unpack_from("<B", report_bytes, offset=2)[0]
	accuracy &= 0b11

	for _offset_idx in range(count):
		total_offset = data_offset + (_offset_idx * 2)
		raw_data = unpack_from(format_str, report_bytes, offset=total_offset)[0]
		scaled_data = raw_data * scalar
		results.append(scaled_data)
	results_tuple = tuple(results)

	return (results_tuple, accuracy)