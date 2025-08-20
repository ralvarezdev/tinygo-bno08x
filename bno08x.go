package go_adafruit_bno08x

import (
	"encoding/binary"
	"fmt"
	"math"
	"time"

	"machine"
)

/*
SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries

SPDX-License-Identifier: MIT
*/

/*
Reset reasons from ID Report response:
0 – Not Applicable
1 – Power On Reset
2 – Internal System Reset
3 – Watchdog Timeout
4 – External Reset
5 – Other
*/

type (
	// PacketReader is an interface for reading packets from the BNO08x sensor
	PacketReader interface {
		ReadPacket() (*Packet, error)
		IsDataReady() bool
	}

	// PacketWriter is an interface for writing packets to the BNO08x sensor
	PacketWriter interface {
		SendPacket(channel uint8, data []byte) (uint8, error)
	}

	// BNO08X struct represents the BNO08x IMU sensor
	BNO08X struct {
		packetReader              PacketReader
		packetWriter              PacketWriter
		debugger                  Debugger
		reset                     *machine.PinOutput
		dataBuffer                DataBuffer
		commandBuffer             []byte
		packetSlices              []*report
		dcdSavedAt                float64
		meCalibrationStartedAt    float64
		calibrationComplete       bool
		magnetometerAccuracy      int
		waitForInitialize         bool
		initComplete              bool
		idRead                    bool
		accelerometer             *[3]float64
		gravity                   *[3]float64
		gyroscope                 *[3]float64
		magnetometer              *[3]float64
		linearAcceleration        *[3]float64
		rotationVector            *[4]float64
		geomagneticRotationVector *[4]float64
		gameRotationVector        *[4]float64
		stepCount                 *uint16
		shakesDetected            *bool
		stabilityClassification   *string
		mostLikelyClassification  *string
		classifications           *map[string]int
		rawAccelerometer          *[3]float64
		rawGyroscope              *[3]float64
		rawMagnetometer           *[3]float64
		enabledFeatures           map[uint8]bool
	}

	// Options struct holds configuration options for the BNO08X instance
	Options struct {
		Debugger Debugger // Debugger instance for debug messages
		Reset    *machine.PinOutput
	}
)

// NewBNO08X creates a new BNO08X instance with the specified reset pin and debug mode
//
// Parameters:
//
//	packetReader: The PacketReader to read packets from the BNO08X sensor.
//	packetWriter: The PacketWriter to write packets to the BNO08X sensor.
//	dataBuffer: The DataBuffer to store Packet data.
//	options: Optional configuration options for the BNO08X instance.
//
// Returns:
//
// A pointer to a new BNO08X instance or an error if initialization fails.
func NewBNO08X(
	packetReader PacketReader,
	packetWriter PacketWriter,
	dataBuffer DataBuffer,
	options *Options,
) (*BNO08X, error) {
	// Check if packetReader, packetWriter and dataBuffer are provided
	if packetReader == nil {
		return nil, ErrNilPacketReader
	}
	if packetWriter == nil {
		return nil, ErrNilPacketWriter
	}
	if dataBuffer == nil {
		return nil, ErrNilDataBuffer
	}

	// If options are nil, initialize with default values
	if options == nil {
		options = &Options{nil, nil}
	}
	bno08x := BNO08X{
		packetReader:              packetReader,
		packetWriter:              packetWriter,
		debugger:                  options.Debugger,
		reset:                     options.Reset,
		dataBuffer:                dataBuffer,
		commandBuffer:             make([]byte, CommandBufferSize),
		packetSlices:              make([]*report, 0),
		dcdSavedAt:                -1.0,
		meCalibrationStartedAt:    -1.0,
		calibrationComplete:       false,
		magnetometerAccuracy:      0,
		waitForInitialize:         true,
		initComplete:              false,
		idRead:                    false,
		accelerometer:             &InitialBnoSensorReportThreeDimensional,
		gravity:                   &InitialBnoSensorReportThreeDimensional,
		gyroscope:                 &InitialBnoSensorReportThreeDimensional,
		magnetometer:              &InitialBnoSensorReportThreeDimensional,
		linearAcceleration:        &InitialBnoSensorReportThreeDimensional,
		rotationVector:            &InitialBnoSensorReportFourDimensional,
		geomagneticRotationVector: &InitialBnoSensorReportFourDimensional,
		gameRotationVector:        &InitialBnoSensorReportFourDimensional,
		stepCount:                 &InitialBnoStepCount,
		shakesDetected:            &InitialBnoShakeDetected,
		stabilityClassification:   &InitialBnoStabilityClassification,
		mostLikelyClassification:  &InitialBnoMostLikelyClassification,
		classifications:           &InitialBnoClassifications,
		rawAccelerometer:          &InitialBnoSensorReportThreeDimensional,
		rawGyroscope:              &InitialBnoSensorReportThreeDimensional,
		rawMagnetometer:           &InitialBnoSensorReportThreeDimensional,
		enabledFeatures:           make(map[uint8]bool),
	}
	bno08x.debug("********** NewBNO08X *************")

	// Initialize the BNO08X sensor
	return &bno08x, bno08x.initialize()
}

// debug prints debug messages if debugging is enabled.
//
// Parameters:
//
//	args: The arguments to print in the debug message
func (b *BNO08X) debug(args ...any) {
	if b.debugger != nil {
		b.debugger.Debug(args...)
	}
}

// hardwareReset performs a hardware reset of the BNO08X sensor to an initial unconfigured state.
func (b *BNO08X) hardwareReset() {
	if b.reset == nil {
		return
	}
	b.debug("Hardware resetting...")
	b.reset.Configure(machine.PinConfig{Mode: machine.PinOutput})

	b.reset.High()
	time.Sleep(10 * time.Millisecond)

	b.reset.Low()
	time.Sleep(10 * time.Millisecond)

	b.reset.High()
	time.Sleep(10 * time.Millisecond)
}

// softwareReset performs a software reset of the BNO08X sensor to an initial unconfigured state.
//
// Returns:
//
// An error if the reset process fails, otherwise nil.
func (b *BNO08X) softwareReset() error {
	b.debug("Software resetting...")
	data := []byte{1}

	if _, err := b.packetWriter.SendPacket(ChannelExe, data); err != nil {
		b.debug("Error sending software reset Packet:", err)
		return err
	}
	time.Sleep(500 * time.Millisecond)

	if _, err := b.packetWriter.SendPacket(ChannelExe, data); err != nil {
		b.debug("Error sending second software reset Packet:", err)
		return err
	}
	time.Sleep(500 * time.Millisecond)

	for i := 0; i < 3; i++ {
		_, err := b.packetReader.ReadPacket()
		if err != nil {
			time.Sleep(500 * time.Millisecond)
		}
	}
	b.debug("OK!")
	return nil
}

// initialize performs the initial setup of the BNO08X sensor, including hardware and software resets.
//
// Returns:
//
// An error if the initialization fails, otherwise nil.
func (b *BNO08X) initialize() error {
	b.debug("Initializing BNO08X sensor...")

	// Check if the sensor ID can be read
	for i := 0; i < 3; i++ {
		// Perform hardware reset
		b.hardwareReset()

		// Perform software reset
		if err := b.softwareReset(); err != nil {
			return err
		}

		ok, err := b.checkID()
		if err != nil {
			return err
		}
		if ok {
			b.initComplete = true
			return nil
		}
		time.Sleep(500 * time.Millisecond)
	}
	return ErrFailedToReadSensorID
}

// checkID checks if the sensor ID can be read from the BNO08X sensor.
//
// Returns:
//
// A boolean indicating whether the sensor ID was successfully read, and an error if there was an issue during the process.
func (b *BNO08X) checkID() (bool, error) {
	b.debug("\n********** READ ID **********")
	if b.idRead {
		return true, nil
	}
	data := make([]byte, 2)
	data[0] = ReportIDSHTPReportProductIDRequest
	data[1] = 0 // padding

	// Send the ID request report
	b.debug("\n** Sending ID Request Report **")
	if _, err := b.packetWriter.SendPacket(ChannelCONTROL, data); err != nil {
		b.debug("Error sending ID request Packet:", err)
		return false, err
	}
	b.debug("\n** Waiting for Packet **")
	for {
		reportID := ReportIDSHTPReportProductIDResponse
		_, err := b.waitForPacketType(ChannelCONTROL, &reportID, nil)
		if err != nil {
			b.debug("Error waiting for ID response Packet:", err)
			return false, err
		}

		// Read the Packet data into the data buffer
		sensorIDReport, err := newReportFromPacketBytes(b.dataBuffer.GetData())
		if err == nil {
			b.debug("Error creating sensor ID report from Packet data")
			continue
		}

		// Parse the sensor ID from the report
		sensorID, err := newSensorID(sensorIDReport)
		if err != nil {
			b.debug("Error parsing sensor ID:", err)
			continue
		}

		b.debug("\n** Sensor ID Report **")
		b.debug(fmt.Sprintf("*** Part Number: %d", sensorID.SoftwarePartNumber))
		b.debug(
			fmt.Sprintf(
				"*** Software Version: %d.%d.%d",
				sensorID.SoftwareMajorVersion,
				sensorID.SoftwareMinorVersion,
				sensorID.SoftwarePatchVersion,
			),
		)
		b.debug(fmt.Sprintf(" Build: %d", sensorID.SoftwareBuildNumber))
		b.idRead = true
		return true, nil
	}
	// unreachable, but for completeness
	// return false
}

// waitForPacketType waits for a Packet of a specific type on a given channel, optionally filtering by report ID.
//
// Parameters:
//
//	channelNumber: The channel number to wait for.
//	reportID: An optional pointer to a report ID to filter packets by.
//	timeout: An optional pointer to a duration to wait before timing out.
//
// Returns:
//
//	A pointer to the Packet if found, or an error if the timeout is reached or an error occurs.
func (b *BNO08X) waitForPacketType(
	channelNumber uint8,
	reportID *uint8,
	timeout *time.Duration,
) (*Packet, error) {
	startTime := time.Now()

	// Check if reportID is provided, and prepare a debug message accordingly
	reportIDStr := ""
	if reportID != nil {
		reportIDStr = fmt.Sprintf(" with report id 0x%X", *reportID)
	}
	b.debug("** Waiting for Packet on channel", channelNumber, reportIDStr)

	// Check if timeout is provided, otherwise set a default
	if timeout == nil {
		timeout = new(time.Duration)
		*timeout = 5 * time.Second // Default timeout of 5 seconds
	}

	for time.Since(startTime) < *timeout {
		// Check if data is ready to be read
		newPacket, err := b.waitForPacket(*timeout - time.Since(startTime))
		if err != nil {
			continue
		}
		if newPacket.ChannelNumber() == channelNumber {
			if reportID != nil {
				if newPacketReportID, _ := newPacket.ReportID(); newPacketReportID == *reportID {
					return newPacket, nil
				}
			} else {
				return newPacket, nil
			}
		}
		if newPacket.ChannelNumber() != ChannelExe && newPacket.ChannelNumber() != ChannelSHTPCommand {
			b.debug("passing Packet to handler for de-slicing")
			if err := b.handlePacket(newPacket); err != nil {
				return nil, err
			}
		}
	}
	return nil, fmt.Errorf(
		"timed out waiting for a Packet on channel %d",
		channelNumber,
	)
}

// waitForPacket waits for a Packet to be available from the Packet reader within the specified timeout.
//
// Parameters:
//
//	timeout: The maximum duration to wait for a Packet.
//
// Returns:
//
//	A pointer to the Packet if available, or an error if the timeout is reached or an error occurs.
func (b *BNO08X) waitForPacket(timeout time.Duration) (*Packet, error) {
	startTime := time.Now()
	for time.Since(startTime) < timeout {
		// Check if data is ready to be read
		if !b.packetReader.IsDataReady() {
			continue
		}

		// Read the Packet from the Packet reader
		newPacket, err := b.packetReader.ReadPacket()
		if err != nil {
			return nil, err
		}
		return newPacket, nil
	}
	return nil, ErrPacketTimeout
}

// handlePacket processes a Packet by separating it into individual reports and processing each report.
//
// Parameters:
//
//	Packet: A pointer to the Packet to be processed.
//
// Returns:
//
//	An error if the Packet cannot be processed, otherwise nil.
func (b *BNO08X) handlePacket(packet *Packet) error {
	// Split out reports first
	if err := separateBatch(packet, &b.packetSlices); err != nil {
		fmt.Println(packet)
		return err
	}

	// Process each report in the Packet slices
	for len(b.packetSlices) > 0 {
		// Pop the last slice
		lastReport := b.packetSlices[len(b.packetSlices)-1]
		b.packetSlices = b.packetSlices[:len(b.packetSlices)-1]

		// Process the report
		if err := b.processReport(lastReport); err != nil {
			b.debug("Error processing report:", err)
			return err
		}
	}
	return nil
}

// processReport processes a report by checking if it is a control report or a sensor report, and then parsing the data accordingly.
//
// Parameters:
//
//	report: A pointer to the report to be processed.
//
// Returns:
//
//	An error if the report cannot be processed, otherwise nil.
func (b *BNO08X) processReport(report *report) error {
	// Check if the report is nil
	if report == nil {
		return ErrNilReport
	}

	if isControlReport(report.ID) {
		return b.handleControlReport(report)
	}

	b.debug("\tProcessing report:", Reports[report.ID])
	if b.debugger != nil {
		outputStr := ""
		for idx, packetByte := range report.Data {
			packetIndex := idx
			if (packetIndex % 4) == 0 {
				outputStr += fmt.Sprintf("\nDBG::\t\t[0x%02X] ", packetIndex)
			}
			outputStr += fmt.Sprintf("0x%02X ", packetByte)
		}
		b.debug(outputStr)
	}

	switch report.ID {
	case ReportIDStepCounter:
		// Parse the step counter report
		stepCounterReport, err := newStepCounterReport(report)
		if err != nil {
			return fmt.Errorf("failed to parse step counter report: %w", err)
		}

		// Update the step count in the BNO08X instance
		b.stepCount = &stepCounterReport.Count
		return nil
	case ReportIDShakeDetector:
		// Parse the shake detector report
		shakeReport, err := newShakeReport(report)
		if err != nil {
			return fmt.Errorf("failed to parse shake report: %w", err)
		}

		// Update the shake detection status in the BNO08X instance
		b.shakesDetected = &shakeReport.AreShakesDetected
		return nil
	case ReportIDStabilityClassifier:
		// Parse the stability classifier report
		stabilityReport, err := newStabilityClassifierReport(report)
		if err != nil {
			return fmt.Errorf(
				"failed to parse stability classifier report: %w",
				err,
			)
		}

		// Update the stability classification in the BNO08X instance
		b.stabilityClassification = &stabilityReport.StabilityClassification
		return nil
	case ReportIDActivityClassifier:
		// Parse the activity classifier report
		activityReport, err := newActivityClassifierReport(report)
		if err != nil {
			return fmt.Errorf(
				"failed to parse activity classifier report: %w",
				err,
			)
		}

		// Update the activity classification and classifications in the BNO08X instance
		b.classifications = &activityReport.Classifications
		return nil
	case ReportIDMagnetometer:
		// Parse the magnetometer report
		magnetometerReport, err := newThreeDimensionalReport(report)
		if err != nil {
			return fmt.Errorf("failed to parse magnetometer report: %w", err)
		}

		// Update the magnetometer readings in the BNO08X instance
		b.magnetometerAccuracy = magnetometerReport.Accuracy
		b.magnetometer = &magnetometerReport.Results
	case ReportIDRotationVector:
		// Parse the rotation vector report
		rotationReport, err := newFourDimensionalReport(report)
		if err != nil {
			return fmt.Errorf("failed to parse rotation vector report: %w", err)
		}

		// Update the rotation vector readings in the BNO08X instance
		b.rotationVector = &rotationReport.Results
	case ReportIDGeomagneticRotationVector:
		// Parse the geomagnetic rotation vector report
		geomagneticReport, err := newFourDimensionalReport(report)
		if err != nil {
			return fmt.Errorf(
				"failed to parse geomagnetic rotation vector report: %w",
				err,
			)
		}

		// Update the geomagnetic rotation vector readings in the BNO08X instance
		b.geomagneticRotationVector = &geomagneticReport.Results
	case ReportIDGameRotationVector:
		// Parse the game rotation vector report
		gameReport, err := newFourDimensionalReport(report)
		if err != nil {
			return fmt.Errorf(
				"failed to parse game rotation vector report: %w",
				err,
			)
		}

		// Update the game rotation vector readings in the BNO08X instance
		b.gameRotationVector = &gameReport.Results
	case ReportIDAccelerometer:
		// Parse the accelerometer report
		accelerometerReport, err := newThreeDimensionalReport(report)
		if err != nil {
			return fmt.Errorf("failed to parse accelerometer report: %w", err)
		}

		// Update the accelerometer readings in the BNO08X instance
		b.accelerometer = &accelerometerReport.Results
	case ReportIDLinearAcceleration:
		// Parse the linear acceleration report
		linearAccelerationReport, err := newThreeDimensionalReport(report)
		if err != nil {
			return fmt.Errorf(
				"failed to parse linear acceleration report: %w",
				err,
			)
		}

		// Update the linear acceleration readings in the BNO08X instance
		b.linearAcceleration = &linearAccelerationReport.Results
	case ReportIDGravity:
		// Parse the gravity report
		gravityReport, err := newThreeDimensionalReport(report)
		if err != nil {
			return fmt.Errorf("failed to parse gravity report: %w", err)
		}

		// Update the gravity readings in the BNO08X instance
		b.gravity = &gravityReport.Results
	case ReportIDGyroscope:
		// Parse the gyroscope report
		gyroscopeReport, err := newThreeDimensionalReport(report)
		if err != nil {
			return fmt.Errorf("failed to parse gyroscope report: %w", err)
		}

		// Update the gyroscope readings in the BNO08X instance
		b.gyroscope = &gyroscopeReport.Results
	case ReportIDRawAccelerometer:
		// Parse the raw accelerometer report
		rawAccelerometerReport, err := newThreeDimensionalReport(report)
		if err != nil {
			return fmt.Errorf(
				"failed to parse raw accelerometer report: %w",
				err,
			)
		}

		// Update the raw accelerometer readings in the BNO08X instance
		b.rawAccelerometer = &rawAccelerometerReport.Results
	case ReportIDRawGyroscope:
		// Parse the raw gyroscope report
		rawGyroscopeReport, err := newThreeDimensionalReport(report)
		if err != nil {
			return fmt.Errorf("failed to parse raw gyroscope report: %w", err)
		}

		// Update the raw gyroscope readings in the BNO08X instance
		b.rawGyroscope = &rawGyroscopeReport.Results
	case ReportIDRawMagnetometer:
		// Parse the raw magnetometer report
		rawMagnetometerReport, err := newThreeDimensionalReport(report)
		if err != nil {
			return fmt.Errorf(
				"failed to parse raw magnetometer report: %w",
				err,
			)
		}

		// Update the raw magnetometer readings in the BNO08X instance
		b.rawMagnetometer = &rawMagnetometerReport.Results
	}

	// If we reach here, the report was processed successfully
	return nil
}

// handleControlReport processes control reports and updates the BNO08X state accordingly.
//
// Parameters:
//
//	report: A pointer to the report containing control data.
//
// Returns:
//
//	An error if the control report cannot be processed, otherwise nil.
func (b *BNO08X) handleControlReport(report *report) error {
	switch report.ID {
	case ReportIDSHTPReportProductIDResponse:
		// Parse the sensor ID from the report bytes
		sensorID, err := newSensorID(report)
		if err != nil {
			return fmt.Errorf("failed to parse sensor ID: %w", err)
		}
		b.debug("FROM PACKET SLICE:")
		b.debug(fmt.Sprintf("*** Part Number: %d", sensorID.SoftwarePartNumber))
		b.debug(
			fmt.Sprintf(
				"*** Software Version: %d.%d.%d",
				sensorID.SoftwareMajorVersion,
				sensorID.SoftwareMinorVersion,
				sensorID.SoftwarePatchVersion,
			),
		)
		b.debug(fmt.Sprintf("\tBuild: %d", sensorID.SoftwareBuildNumber))
	case ReportIDGetFeatureResponse:
		// Parse the Get Feature report from the report bytes
		_, err := newGetFeatureReport(report)
		if err != nil {
			return fmt.Errorf("failed to parse get feature report: %w", err)
		}
	case ReportIDCommandResponse:
		return b.handleCommandResponse(report)
	}
	return nil
}

// handleCommandResponse processes the command response report and updates the BNO08X state accordingly.
//
// Parameters:
//
//	report: A pointer to the report containing command response data.
//
// Returns:
//
//	An error if the command response cannot be processed, otherwise nil.
func (b *BNO08X) handleCommandResponse(report *report) error {
	commandResponse, err := newCommandResponse(report)
	if err != nil {
		return err
	}

	// Get the command and its status from the command response
	command := commandResponse.Command
	commandStatus := commandResponse.Status()

	if command == MECalibrate && commandStatus == 0 {
		b.meCalibrationStartedAt = float64(time.Now().UnixNano()) / 1e9
	}

	if command == SaveDCD {
		if commandStatus == 0 {
			b.dcdSavedAt = float64(time.Now().UnixNano()) / 1e9
		} else {
			return ErrFailedToSaveCalibrationData
		}
	}
	return nil
}

// processAvailablePackets processes all available packets from the Packet reader, handling each Packet until the maximum number of packets is reached.
//
// Parameters:
//
//	maxPackets: An optional pointer to an integer specifying the maximum number of packets to process. If nil, all available packets will be processed.
func (b *BNO08X) processAvailablePackets(maxPackets *int) {
	processedCount := 0
	for b.packetReader.IsDataReady() {
		if maxPackets != nil && processedCount >= *maxPackets {
			return
		}
		newPacket, err := b.packetReader.ReadPacket()
		if err != nil {
			continue
		}
		if err := b.handlePacket(newPacket); err != nil {
			b.debug("Error handling Packet:", err)
			continue
		}
		processedCount++
	}
}

// Magnetic returns the current magnetic field measurements on the X, Y, and Z axes.
//
// Returns:
//
// A pointer to a [3]float64 array containing the magnetic field values.
func (b *BNO08X) Magnetic() *[3]float64 {
	// Process available packets to ensure readings are up-to-date
	b.processAvailablePackets(nil)
	return b.magnetometer
}

// Quaternion returns a pointer to a [4]float64 array representing the current rotation vector as a quaternion.
//
// Returns:
//
// A pointer to a [4]float64 array containing the quaternion values.
func (b *BNO08X) Quaternion() *[4]float64 {
	// Process available packets to ensure readings are up-to-date
	b.processAvailablePackets(nil)
	return b.rotationVector
}

// QuaternionEulerDegrees returns the current rotation vector as Euler angles in degrees.
//
// Returns:
//
// A tuple of three float64 values representing the roll, pitch, and yaw angles in degrees.
func (b *BNO08X) QuaternionEulerDegrees() *[3]float64 {
	// Get the quaternion readings
	b.Quaternion()

	// Get the quaternion components
	x := b.rotationVector[0]
	y := b.rotationVector[1]
	z := b.rotationVector[2]
	w := b.rotationVector[3]

	// Roll (X axis)
	sinRollCosPitch := 2 * (w*x + y*z)
	cosRollCosPitch := 1 - 2*(x*x+y*y)
	roll := math.Atan2(sinRollCosPitch, cosRollCosPitch)

	// Pitch (Y axis)
	sinPitch := 2 * (w*y - z*x)
	var pitch float64
	if sinPitch >= 1 {
		pitch = math.Pi / 2
	} else if sinPitch <= -1 {
		pitch = -math.Pi / 2
	} else {
		pitch = math.Asin(sinPitch)
	}

	// Yaw (Z axis)
	sinYawCosPitch := 2 * (w*z + x*y)
	cosYawCosPitch := 1 - 2*(y*y+z*z)
	yaw := math.Atan2(sinYawCosPitch, cosYawCosPitch)

	return &[3]float64{
		roll * 180 / math.Pi,
		pitch * 180 / math.Pi,
		yaw * 180 / math.Pi,
	}
}

// GeomagneticQuaternion returns a pointer to a [4]float64 array representing the current geomagnetic rotation vector as a quaternion.
//
// Returns:
//
// A pointer to a [4]float64 array containing the geomagnetic quaternion values.
func (b *BNO08X) GeomagneticQuaternion() *[4]float64 {
	// Process available packets to ensure readings are up-to-date
	b.processAvailablePackets(nil)
	return b.geomagneticRotationVector
}

// GameQuaternion returns a pointer to a [4]float64 array representing the current rotation vector expressed as a quaternion with no specific reference for heading.
//
// Returns:
//
// A pointer to a [4]float64 array containing the game quaternion values.
func (b *BNO08X) GameQuaternion() *[4]float64 {
	// Process available packets to ensure readings are up-to-date
	b.processAvailablePackets(nil)
	return b.gameRotationVector
}

// Steps returns the number of steps detected since the sensor was initialized.
//
// Returns:
//
//	A pointer to an uint16 representing the step count.
func (b *BNO08X) Steps() *uint16 {
	b.processAvailablePackets(nil)
	return b.stepCount
}

// LinearAcceleration returns the current linear acceleration values on the X, Y, and Z axes in meters per second squared.
//
// Returns:
//
//	A pointer to a [3]float64 array containing the linear acceleration values.
func (b *BNO08X) LinearAcceleration() *[3]float64 {
	b.processAvailablePackets(nil)
	return b.linearAcceleration
}

// Acceleration returns the acceleration measurements on the X, Y, and Z axes in meters per second squared.
//
// Returns:
//
//	A pointer to a [3]float64 array containing the acceleration values.
func (b *BNO08X) Acceleration() *[3]float64 {
	b.processAvailablePackets(nil)
	return b.accelerometer
}

// Gravity returns the gravity vector in the X, Y, and Z components in meters per second squared.
//
// Returns:
//
//	A pointer to a [3]float64 array containing the gravity vector.
func (b *BNO08X) Gravity() *[3]float64 {
	b.processAvailablePackets(nil)
	return b.gravity
}

// Gyro returns Gyro's rotation measurements on the X, Y, and Z axes in radians per second.
//
// Returns:
//
//	A pointer to a [3]float64 array containing the gyroscope values.
func (b *BNO08X) Gyro() *[3]float64 {
	b.processAvailablePackets(nil)
	return b.gyroscope
}

// GyroDegrees returns Gyro's rotation measurements on the X, Y, and Z axes in degrees per second.
//
// Returns:
//
// A pointer to a [3]float64 array containing the gyroscope values in degrees.
func (b *BNO08X) GyroDegrees() *[3]float64 {
	// Get the gyroscope readings
	b.Gyro()

	// Convert radians to degrees
	gyroDegrees := [3]float64{
		b.gyroscope[0] * (180.0 / math.Pi),
		b.gyroscope[1] * (180.0 / math.Pi),
		b.gyroscope[2] * (180.0 / math.Pi),
	}
	return &gyroDegrees
}

// Shake returns true if a shake was detected on any axis since the last time it was checked.
// This method has a latching behavior: once a shake is detected, it stays "shaken" until read.
//
// Returns:
//
//	A pointer to a bool indicating if a shake was detected.
func (b *BNO08X) Shake() *bool {
	b.processAvailablePackets(nil)
	if b.shakesDetected != nil && *b.shakesDetected {
		*b.shakesDetected = false // clear on read
		return b.shakesDetected
	}
	return b.shakesDetected
}

// StabilityClassification returns the sensor's assessment of its current stability.
//
// Returns:
//
//	A pointer to a string describing the stability classification.
func (b *BNO08X) StabilityClassification() *string {
	b.processAvailablePackets(nil)
	return b.stabilityClassification
}

// ActivityClassification returns the sensor's assessment of the activity creating the sensed motions.
//
// Returns:
//
//	A pointer to a map[string]int representing activity classifications.
func (b *BNO08X) ActivityClassification() *map[string]int {
	b.processAvailablePackets(nil)
	return b.classifications
}

// RawAcceleration returns the sensor's raw, unscaled value from the accelerometer registers.
//
// Returns:
//
//	A pointer to a [3]float64 array containing the raw accelerometer values.
func (b *BNO08X) RawAcceleration() *[3]float64 {
	b.processAvailablePackets(nil)
	return b.rawAccelerometer
}

// RawGyro returns the sensor's raw, unscaled value from the gyro registers.
//
// Returns:
//
//	A pointer to a [3]float64 array containing the raw gyroscope values.
func (b *BNO08X) RawGyro() *[3]float64 {
	b.processAvailablePackets(nil)
	return b.rawGyroscope
}

// RawMagnetic returns the sensor's raw, unscaled value from the magnetometer registers.
//
// Returns:
//
//	A pointer to a [3]float64 array containing the raw magnetometer values.
func (b *BNO08X) RawMagnetic() *[3]float64 {
	b.processAvailablePackets(nil)
	return b.rawMagnetometer
}

// getFeatureEnableReport creates a Packet to enable a feature on the BNO08X sensor.
//
// Parameters:
//
//	featureID: The ID of the feature to enable.
//	reportInterval: The interval for reporting (default: DefaultReportInterval).
//	sensorSpecificConfig: Sensor-specific configuration bits.
//
// Returns:
//
//	A byte slice representing the feature enable report.
func getFeatureEnableReport(
	featureID uint8,
	reportInterval uint32,
	sensorSpecificConfig uint32,
) []byte {
	setFeatureReport := make([]byte, 17)
	setFeatureReport[0] = ReportIDSetFeatureCommand
	setFeatureReport[1] = featureID
	binary.LittleEndian.PutUint32(setFeatureReport[5:], reportInterval)
	binary.LittleEndian.PutUint32(setFeatureReport[13:], sensorSpecificConfig)
	return setFeatureReport
}

// EnableFeature enables a given feature of the BNO08X sensor.
//
// Parameters:
//
//	featureID: The ID of the feature to enable.
//
// Returns:
//
//	An error if the feature could not be enabled.
func (b *BNO08X) EnableFeature(featureID uint8) error {
	b.debug("\n********** Enabling feature id:", featureID, "**********")

	// Create the feature enable report based on the feature ID
	var setFeatureReport []byte
	if featureID == ReportIDActivityClassifier {
		setFeatureReport = getFeatureEnableReport(
			featureID,
			DefaultReportInterval,
			EnabledActivities,
		)
	} else {
		setFeatureReport = getFeatureEnableReport(
			featureID,
			DefaultReportInterval,
			0,
		)
	}

	// Check if the feature has a dependency
	featureDependency, ok := RawReports[featureID]
	if ok && b.isFeatureEnabled(featureDependency) {
		b.debug("Enabling feature dependency:", featureDependency)
		if err := b.EnableFeature(featureDependency); err != nil {
			return err
		}
	}

	b.debug("Enabling", featureID)
	if _, err := b.packetWriter.SendPacket(
		ChannelCONTROL,
		setFeatureReport,
	); err != nil {
		return err
	}

	startTime := time.Now()
	for time.Since(startTime) < FeatureEnableTimeout {
		maxPackets := 10
		b.processAvailablePackets(&maxPackets)

		// Check if the feature is enabled (update this check as needed for your readings map)
		if b.isFeatureEnabled(featureID) {
			return nil
		}
	}
	return fmt.Errorf("was not able to enable feature %d", featureID)
}

// Is FeatureEnabled checks if a specific feature is enabled on the BNO08X sensor.
//
// Parameters:
//
//	featureID: The ID of the feature to check.
//
// Returns:
//
//	A boolean indicating whether the feature is enabled.
func (b *BNO08X) isFeatureEnabled(featureID uint8) bool {
	_, ok := b.enabledFeatures[featureID]
	return ok
}

// BeginCalibration starts the self-calibration routine for the BNO08X sensor.
func (b *BNO08X) BeginCalibration() {
	// Begin the sensor's self-calibration routine
	params := []byte{
		1, // calibrate accel
		1, // calibrate gyro
		1, // calibrate mag
		MECalibrationConfig,
		0, // calibrate planar acceleration
		0, // 'on_table' calibration
		0, // reserved
		0, // reserved
		0, // reserved
	}
	b.sendMeCommand(params)
	b.calibrationComplete = false
}

// CalibrationStatus retrieves the status of the self-calibration process.
//
// Returns:
//
// An integer representing the calibration status, where 0 indicates no calibration needed,
func (b *BNO08X) CalibrationStatus() int {
	// Get the status of the self-calibration
	params := []byte{
		0, // calibrate accel
		0, // calibrate gyro
		0, // calibrate mag
		MEGetCalibration,
		0, // calibrate planar acceleration
		0, // 'on_table' calibration
		0, // reserved
		0, // reserved
		0, // reserved
	}
	b.sendMeCommand(params)
	return b.magnetometerAccuracy
}

// sendMeCommand sends a command to the BNO08X sensor using the ME command protocol.
//
// Parameters:
//
//	subcommandParams: A byte slice containing the parameters for the command.
func (b *BNO08X) sendMeCommand(subcommandParams []byte) {
	startTime := time.Now()
	localBuffer := b.commandBuffer

	// Insert the command request report into the local buffer
	err := insertCommandRequestReport(
		MECalibrate,
		&b.commandBuffer, // should use b.dataBuffer, but sendPacket doesn't
		b.dataBuffer.GetReportSequenceNumber(ReportIDCommandRequest),
		&subcommandParams,
	)
	if err != nil {
		b.debug("Error inserting command request report:", err)
		return
	}

	// Send the command request Packet
	_, err = b.packetWriter.SendPacket(ChannelCONTROL, localBuffer)
	if err != nil {
		b.debug("Error sending command request Packet:", err)
		return
	}
	b.dataBuffer.IncrementReportSequenceNumber(ReportIDCommandRequest)

	// Wait for the command response
	for time.Since(startTime) < DefaultTimeout {
		b.processAvailablePackets(nil)
		if b.meCalibrationStartedAt > float64(startTime.UnixNano())/1e9 {
			break
		}
	}
}

// SaveCalibrationData saves the self-calibration data to the BNO08X sensor.
//
// Returns:
//
// An error if the calibration data could not be saved, otherwise nil.
func (b *BNO08X) SaveCalibrationData() error {
	// Save the self-calibration data
	startTime := time.Now()
	localBuffer := make([]byte, 12)
	err := insertCommandRequestReport(
		SaveDCD,
		&localBuffer, // should use b.dataBuffer, but sendPacket doesn't
		b.dataBuffer.GetReportSequenceNumber(ReportIDCommandRequest),
		nil,
	)
	if err != nil {
		return err
	}

	// Send the command request Packet to save calibration data
	_, err = b.packetWriter.SendPacket(ChannelCONTROL, localBuffer)
	if err != nil {
		return err
	}
	b.dataBuffer.IncrementReportSequenceNumber(ReportIDCommandRequest)

	// Wait for the command response indicating that the calibration data was saved
	for time.Since(startTime) < DefaultTimeout {
		b.processAvailablePackets(nil)
		if b.dcdSavedAt > float64(startTime.UnixNano())/1e9 {
			return nil
		}
	}
	return ErrFailedToSaveCalibrationData
}
