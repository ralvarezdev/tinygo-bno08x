package tinygo_bno08x

import (
	"time"

	"machine"

	tinygoerrors "github.com/ralvarezdev/tinygo-errors"
	tinygologger "github.com/ralvarezdev/tinygo-logger"
	tinygobuffers "github.com/ralvarezdev/tinygo-buffers"
)

type (
	// UARTRVC is a the UART-RVC implementation for the BNO08x sensor.
	UARTRVC struct {
		uartBus       *machine.UART
		txPin         machine.Pin
		rxPin         machine.Pin
		ps0Pin        machine.Pin
		ps1Pin        machine.Pin
		resetPin      machine.Pin
		logger      tinygologger.Logger
		timeout       time.Duration
		accelerometer [3]float64
		eulerDegrees  [3]float64
		initComplete  bool
		buffer 	  []byte
	}
)

var (
	// invalidChecksumMessage is the message printed when an invalid checksum is detected
	invalidChecksumMessage = []byte("Invalid checksum detected")

	// receivedFrameMessage is the message printed when a frame is received
	receivedFrameMessage = []byte("Received frame")

	// failedToParseFrameMessage is the message printed when parsing a frame fails
	failedToParseFrameMessage = []byte("Failed to parse frame")
)

// NewUARTRVC creates a new UARTRVC instance.
//
// Parameters:
//
// uartBus: The UART bus to use for communication.
// txPin: The TX pin for UART communication.
// rxPin: The RX pin for UART communication.
// ps0Pin: The PS0 pin to set the sensor to UART mode.
// ps1Pin: The PS1 pin to set the sensor to UART mode.
// resetPin: The pin used to reset the BNO08X sensor.
// logger: The logger to use for logging and debugging information (optional).
//
// Returns:
//
// An instance of UARTRVC, or an error if the uartBus is nil or configuration fails.
func NewUARTRVC(
	uartBus *machine.UART,
	txPin machine.Pin,
	rxPin machine.Pin,
	ps0Pin machine.Pin,
	ps1Pin machine.Pin,
	resetPin machine.Pin,
	logger tinygologger.Logger,
) (*UARTRVC, tinygoerrors.ErrorCode) {
	// Check if the UART bus is nil
	if uartBus == nil {
		return nil, ErrorCodeBNO08XNilUARTBus
	}

	// Set PS0 pin to output and high
	ps0Pin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	ps0Pin.High()

	// Set PS1 pin to output and low
	ps1Pin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	ps1Pin.Low()

	// Configure UART
	if err := uartBus.Configure(
		machine.UARTConfig{
			BaudRate: UARTRVCBaudRate,
			TX:       txPin,
			RX:       rxPin,
		},
	); err != nil {
		return nil, ErrorCodeBNO08XFailedToConfigureUART
	}

	// Create the UART-RVC instance
	uartRVC := &UARTRVC{
		uartBus:       uartBus,
		txPin:         txPin,
		rxPin:         rxPin,
		ps0Pin:        ps0Pin,
		ps1Pin:        ps1Pin,
		resetPin:      resetPin,
		logger:      logger,
		buffer:        make([]byte, UARTRVCPacketLengthBytes),
		timeout:       UARTRVCTimeout,
		initComplete:  false,
	}

	// Perform reset
	if err := uartRVC.Reset(); err != tinygoerrors.ErrorCodeNil {
		return nil, ErrorCodeBNO08XFailedToResetUARTRVC
	}
	return uartRVC, tinygoerrors.ErrorCodeNil
}

// Reset performs a hardware reset of the BNO08X sensor using the specified reset pin.
//
// Returns:
//
// An error if the reset process fails, otherwise nil.
func (u *UARTRVC) Reset() tinygoerrors.ErrorCode  {
	HardwareReset(u.resetPin, u.logger)
	return tinygoerrors.ErrorCodeNil
}

// ParseFrame parses a heading frame from the BNO08x RVC.
//
// Returns:
//
// An error if parsing fails.
func (u *UARTRVC) ParseFrame() tinygoerrors.ErrorCode {
	// Retrieve and parse fields from the frame
	// index := u.buffer[2
	yaw, _ := tinygobuffers.BytesToUint16LE(u.buffer[3:5])
	pitch, _ := tinygobuffers.BytesToUint16LE(u.buffer[5:7])
	roll, _ := tinygobuffers.BytesToUint16LE(u.buffer[7:9])
	xAccel, _ := tinygobuffers.BytesToUint16LE(u.buffer[9:11])
	yAccel, _ := tinygobuffers.BytesToUint16LE(u.buffer[11:13])
	zAccel, _ := tinygobuffers.BytesToUint16LE(u.buffer[13:15])
	// res1 := u.buffer[15]
	// res2 := u.buffer[16]
	// res3 := u.buffer[17]
	checksum := u.buffer[18]

	// Validate checksum
	checksumCalc := byte(0)
	for i := UARTRVCHeaderLength; i < UARTRVCPacketLengthBytes-1; i++ {
		checksumCalc += u.buffer[i]
	}
	if checksumCalc != checksum {
		if u.logger != nil {
			u.logger.InfoMessage(invalidChecksumMessage)
		}
		return ErrorCodeBNO08XUARTRVCInvalidChecksum
	}

	// Update the current euler degrees
	u.eulerDegrees[EulerDegreesYawIndex] = float64(yaw) * 0.01
	u.eulerDegrees[EulerDegreesPitchIndex] = float64(pitch) * 0.01
	u.eulerDegrees[EulerDegreesRollIndex] = float64(roll) * 0.01

	// Update the current accelerometer values
	u.accelerometer[ThreeDimensionalXIndex] = float64(xAccel) * MilligToMeterPerSecondSquared
	u.accelerometer[ThreeDimensionalYIndex] = float64(yAccel) * MilligToMeterPerSecondSquared
	u.accelerometer[ThreeDimensionalZIndex] = float64(zAccel) * MilligToMeterPerSecondSquared
	return tinygoerrors.ErrorCodeNil
}

// readByte blocks until a byte is read (simple poll).
//
// Returns:
//
// A byte read from UART and an error if any.
func (u *UARTRVC) readByte() (byte, tinygoerrors.ErrorCode) {
	startTime := time.Now()
	for time.Since(startTime) < UARTByteTimeout {
		if u.uartBus.Buffered() > 0 {
			if b, err := u.uartBus.ReadByte(); err == nil {
				return b, tinygoerrors.ErrorCodeNil
			}
			return 0, ErrorCodeBNO08XUARTRVCFailedToReadByte
		}
	}
	return 0, ErrorCodeBNO08XUARTRVCByteTimeout
}

// Read reads a heading frame from the BNO08x from UART-RVC.
//
// Returns:
//
// An error if reading fails or times out.
func (u *UARTRVC) Read() tinygoerrors.ErrorCode {
	// Clear frame buffer
	for i := range u.buffer {
		u.buffer[i] = 0
	}

	// Get the start time for timeout calculation
	start := time.Now()
	for time.Since(start) < u.timeout {
		// Loop until timeout to find the start bytes
		processedBytes := 0
		for processedBytes < UARTRVCHeaderLength {
			b, err := u.readByte()
			if err != tinygoerrors.ErrorCodeNil {
				return err
			}
			u.buffer[processedBytes] = b
			processedBytes++
		}

		// Check if we have the start bytes
		if u.buffer[0] != UARTRVCStartByte || u.buffer[1] != UARTRVCStartByte {
			// Check if we have the first start byte
			if u.buffer[1] == UARTRVCStartByte {
				// Shift the second byte to the first position
				u.buffer[0] = u.buffer[1]
				processedBytes = 1
			} else {
				// Reset processed bytes
				processedBytes = 0
			}
			continue
		}

		// We have the start bytes, read the rest of the frame
		for processedBytes < UARTRVCPacketLengthBytes {
			b, err := u.readByte()
			if err != tinygoerrors.ErrorCodeNil {
				return err
			}
			u.buffer[processedBytes] = b
			processedBytes++
		}

		// Print the raw frame for debugging
		if u.logger != nil {
			u.logger.InfoMessage(receivedFrameMessage)
		}

		// Parse the frame
		if err := u.ParseFrame(); err != tinygoerrors.ErrorCodeNil {
			if u.logger != nil {
				u.logger.WarningMessage(failedToParseFrameMessage)
			}
			return ErrorCodeBNO08XFailedToParseFrame
		}
		return tinygoerrors.ErrorCodeNil
	}
	return ErrorCodeBNO08XUARTRVCUARTTimeout
}

// Update reads a new frame from the BNO08x sensor and updates the internal state.
//
// Returns:
//
// An error if reading or parsing the frame fails.
func (u *UARTRVC) Update() tinygoerrors.ErrorCode {
	return u.Read()
}

// GetAcceleration returns the acceleration measurements on the X, Y, and Z axes in meters per second squared.
//
// Returns:
//
//	A [3]float64 array containing the acceleration values.
func (u *UARTRVC) GetAcceleration() [3]float64 {
	return u.accelerometer
}

// GetEulerDegrees returns the current rotation vector as Euler angles in degrees.
//
// Returns:
//
// A tuple of three float64 values representing the roll, pitch, and yaw angles in degrees.
func (u *UARTRVC) GetEulerDegrees() [3]float64 {
	return u.eulerDegrees
}
