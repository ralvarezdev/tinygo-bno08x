package go_adafruit_bno055

import "errors"

var (
	ErrNilPacketBytes                       = errors.New("nil packet bytes provided for parsing")
	ErrNilReportData                        = errors.New("nil report data provided for parsing")
	ErrNilReportBytes                       = errors.New("nil report bytes provided for parsing")
	ErrNilBuffer                            = errors.New("nil buffer provided for reading data")
	ErrPacketParse                          = errors.New("packet could not be parsed")
	ErrReportBytesTooShort                  = errors.New("report bytes are too short to parse")
	ErrStabilityClassifierTooShort          = errors.New("stability classifier report bytes are too short to parse")
	ErrBufferTooShort                       = errors.New("buffer is too short to read the expected data")
	ErrBufferTooShortForHeader              = errors.New("buffer is too short to read the packet header")
	ErrPacketDataTooShort                   = errors.New("packet data is too short to read the expected data")
	ErrInvalidReportIDForSensorID           = errors.New("invalid report ID for sensor ID parsing")
	ErrInvalidReportIDForActivityClassifier = errors.New("invalid report ID for activity classifier parsing")
	ErrInvalidReportIDForCommandResponse    = errors.New("invalid report ID for command response parsing")
	ErrUnknownReportID                      = errors.New("unknown report ID received from sensor")
	ErrNilSensorReport                      = errors.New("nil sensor report provided for parsing")
	ErrNotImplemented                       = errors.New("function not implemented yet")
	ErrCommandRequestTooManyArguments       = errors.New("command request cannot have more than 9 arguments")
	ErrFailedToReadSensorID                 = errors.New("failed to read sensor ID from the device")
	ErrNilPacketReader                      = errors.New("nil packet header provided for parsing")
	ErrNilPacketWriter                      = errors.New("nil packet writer provided for writing data")
	ErrPacketTimeout                        = errors.New("packet read timeout exceeded")
	ErrNilPacket                            = errors.New("nil packet provided for processing")
)
