//go:build tinygo && (rp2040 || rp2350)

package go_adafruit_bno08x

import "errors"

var (
	ErrNilPacketBytes                            = errors.New("nil Packet bytes provided for parsing")
	ErrNilReport                                 = errors.New("nil report provided for parsing")
	ErrNilReportData                             = errors.New("nil report data provided for parsing")
	ErrNilBuffer                                 = errors.New("nil buffer provided for reading data")
	ErrNilPacketReader                           = errors.New("packet reader is nil, cannot read data")
	ErrNilPacketWriter                           = errors.New("packet writer is nil, cannot write data")
	ErrNilDataBuffer                             = errors.New("nil data buffer provided for reading data")
	ErrReportDataTooShort                        = errors.New("report data are too short to parse")
	ErrStabilityClassifierTooShort               = errors.New("stability classifier report bytes are too short to parse")
	ErrBufferTooShort                            = errors.New("buffer is too short to read the expected data")
	ErrBufferTooShortForHeader                   = errors.New("buffer is too short to read the Packet header")
	ErrPacketDataTooShort                        = errors.New("packet data is too short to read the expected data")
	ErrInvalidReportIDForReportParsing           = "invalid report ID for report parsing, expected %d, got %d"
	ErrUnknownReportID                           = errors.New("unknown report ID received from sensor")
	ErrNilSensorReport                           = errors.New("nil sensor report provided for parsing")
	ErrCommandRequestTooManyArguments            = errors.New("command request cannot have more than 9 arguments")
	ErrFailedToReadSensorID                      = errors.New("failed to read sensor ID from the device")
	ErrPacketTimeout                             = errors.New("packet read timeout exceeded")
	ErrNilPacket                                 = errors.New("nil Packet provided for processing")
	ErrFailedToSaveCalibrationData               = errors.New("failed to save calibration data to the device")
	ErrInvalidReportIDForThreeDimensionalParsing = errors.New("invalid report ID for three-dimensional parsing")
	ErrInvalidReportIDForFourDimensionalParsing  = errors.New("invalid report ID for four-dimensional parsing")
	ErrInvalidChannel                            = errors.New("invalid channel number provided")
	ErrNoPacketAvailable                         = errors.New("no Packet available to read from the I2C bus")
)
