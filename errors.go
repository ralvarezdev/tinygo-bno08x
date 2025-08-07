package go_adafruit_bno055

import "errors"

var (
	ErrPacketParse                          = errors.New("packet could not be parsed")
	ErrReportBytesTooShort                  = errors.New("report bytes are too short to parse")
	ErrStabilityClassifierTooShort          = errors.New("stability classifier report bytes are too short to parse")
	ErrBufferTooShort                       = errors.New("buffer is too short to read the expected data")
	ErrInvalidReportIDForSensorID           = errors.New("invalid report ID for sensor ID parsing")
	ErrInvalidReportIDForActivityClassifier = errors.New("invalid report ID for activity classifier parsing")
	ErrInvalidReportIDForCommandResponse    = errors.New("invalid report ID for command response parsing")
	ErrUnknownReportID                      = errors.New("unknown report ID received from sensor")
	ErrNilSensorReport                      = errors.New("nil sensor report provided for parsing")
)
