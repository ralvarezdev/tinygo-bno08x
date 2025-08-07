package go_adafruit_bno055

import "errors"

var (
	ErrPacketParse         = errors.New("packet could not be parsed")
	ErrReportBytesTooShort = errors.New("report bytes are too short to parse")
)
