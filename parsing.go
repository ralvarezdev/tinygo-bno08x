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

func _parse_stability_classifier_report(report_bytes: bytearray) -> str:
	classification_bitfield = unpack_from("<B", report_bytes, offset=4)[0]
	return ["Unknown", "On Table", "Stationary", "Stable", "In motion"][classification_bitfield]

// 0 Report ID = 0x1E
// 1 Sequence number
// 2 Status
// 3 Delay
// 4 Page Number + EOS
// 5 Most likely state
// 6-15 Classification (10 x Page Number) + confidence
func _parse_activity_classifier_report(report_bytes: bytearray) -> dict[str, str]:
	activities = [
	"Unknown",
	"In-Vehicle",  // look
	"On-Bicycle",  // at
	"On-Foot",  // all
	"Still",  // this
	"Tilting",  // room
	"Walking",  // for
	"Running",  // activities
	"OnStairs",
	]

	end_and_page_number = unpack_from("<B", report_bytes, offset=4)[0]
	// last_page = (end_and_page_number & 0b10000000) > 0
	page_number = end_and_page_number & 0x7F
	most_likely = unpack_from("<B", report_bytes, offset=5)[0]
	confidences = unpack_from("<BBBBBBBBB", report_bytes, offset=6)

	classification = {}
	classification["most_likely"] = activities[most_likely]
	for idx, raw_confidence in enumerate(confidences):
	confidence = (10 * page_number) + raw_confidence
	activity_string = activities[idx]
	classification[activity_string] = confidence
	return classification

func parse_sensor_id(buffer: bytearray) -> tuple[int, ...]:
	"""Parse the fields of a product id report"""
	if not buffer[0] == SHTPReportProductIdResponse:
	raise AttributeError("Wrong report id for sensor id: %s" % hex(buffer[0]))

	sw_major = unpack_from("<B", buffer, offset=2)[0]
	sw_minor = unpack_from("<B", buffer, offset=3)[0]
	sw_patch = unpack_from("<H", buffer, offset=12)[0]
	sw_part_number = unpack_from("<I", buffer, offset=4)[0]
	sw_build_number = unpack_from("<I", buffer, offset=8)[0]

	return (sw_part_number, sw_major, sw_minor, sw_patch, sw_build_number)

