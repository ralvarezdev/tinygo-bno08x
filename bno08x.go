package go_adafruit_bno055

import (
	"encoding/binary"
	"errors"
	"fmt"
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
		ReadPacket() (*packet, error)
		IsDataReady() bool
	}

	// PacketWriter is an interface for writing packets to the BNO08x sensor
	PacketWriter interface {
		SendPacket(channel uint8, data []byte) (int, error)
	}

	// BNO08X struct represents the BNO08x IMU sensor
	BNO08X struct {
		packetReader          PacketReader
		packetWriter          PacketWriter
		debugFlag               bool
		reset                   *machine.PinOutput
		dataBuffer              []byte
		commandBuffer           []byte
		packetSlices            []*report
		sequenceNumber          []int
		twoEndedSequenceNumbers map[int]int
		dcdSavedAt              float64
		meCalibrationStartedAt  float64
		calibrationComplete     bool
		magnetometerAccuracy    int
		waitForInitialize       bool
		initComplete            bool
		idRead                  bool
		accelerometer *[3]float64
		gravity *[3]float64
		gyroscope *[3]float64
		magnetometer *[3]float64
		linearAcceleration *[3]float64
		rotationVector *[4]float64
		geomagneticRotationVector *[4]float64
		gameRotationVector *[4]float64
		stepCount *uint16
		shakesDetected *bool
		stabilityClassification *string
		mostLikelyClassification *string
		classifications *map[string]int
		rawAccelerometer *[3]float64
		rawGyroscope *[3]float64
		rawMagnetometer *[3]float64
	}

	// Options struct holds configuration options for the BNO08X instance
	Options struct {
		Debug bool
		Reset *machine.PinOutput
	}
)

// NewBNO08X creates a new BNO08X instance with the specified reset pin and debug mode
func NewBNO08X(packetReader PacketReader, packetWriter PacketWriter, options *Options) (*BNO08X, error) {
	// Check if packetReader and packetWriter are provided
	if packetReader == nil {
		return nil, ErrNilPacketReader
	}
	if packetWriter == nil {
		return nil, ErrNilPacketWriter
	}

	// If options are nil, initialize with default values
	if options == nil {
		options = &Options{true, nil}
	}
	bno08x := BNO08X{
		packetReader:          packetReader,
		packetWriter:          packetWriter,
		debugFlag:               options.Debug,
		reset:                   options.Reset,
		dataBuffer:              make([]byte, DataBufferSize),
		commandBuffer:           make([]byte, CommandBufferSize),
		packetSlices:            make([]*report, 0),
		sequenceNumber:          make([]int, 6), // Assuming 6 channels
		twoEndedSequenceNumbers: make(map[int]int),
		dcdSavedAt:              -1.0,
		meCalibrationStartedAt:  -1.0,
		calibrationComplete:     false,
		magnetometerAccuracy:    0,
		waitForInitialize:       true,
		initComplete:            false,
		idRead:                  false,
		accelerometer: &InitialBnoSensorReportThreeDimensional,
		gravity: &InitialBnoSensorReportThreeDimensional,
		gyroscope: &InitialBnoSensorReportThreeDimensional,
		magnetometer: &InitialBnoSensorReportThreeDimensional,
		linearAcceleration: &InitialBnoSensorReportThreeDimensional,
		rotationVector: &InitialBnoSensorReportFourDimensional,
		geomagneticRotationVector: &InitialBnoSensorReportFourDimensional,
		gameRotationVector: &InitialBnoSensorReportFourDimensional,
		stepCount: &InitialBnoStepCount,
		shakesDetected: &InitialBnoShakeDetected,
		stabilityClassification: &InitialBnoStabilityClassification,
		mostLikelyClassification: &InitialBnoMostLikelyClassification,
		classifications: &InitialBnoClassifications,
		rawAccelerometer: &InitialBnoSensorReportThreeDimensional,
		rawGyroscope: &InitialBnoSensorReportThreeDimensional,
		rawMagnetometer: &InitialBnoSensorReportThreeDimensional,
	}
	bno08x.debug("********** NewBNO08X *************")

	// Initialize the BNO08X sensor
	return &bno08x, bno08x.initialize()
}

// debug function to print debug messages
//
// Parameters:
//
//	args: The arguments to print in the debug message
func (b *BNO08X) debug(args ...any) {
	if b.debugFlag {
		fmt.Print("DBG::\t\t")
		fmt.Print( args...)
		fmt.Println()
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

	if _, err :=b.packetWriter.SendPacket(ChannelExe, data); err != nil {
		b.debug("Error sending software reset packet:", err)
		return err
	}
	time.Sleep(500 * time.Millisecond)

	if _, err := b.packetWriter.SendPacket(ChannelExe, data); err != nil {
		b.debug("Error sending second software reset packet:", err)
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
		b.debug("Error sending ID request packet:", err)
		return false, err
	}
	b.debug("\n** Waiting for packet **")
	for {
		reportID := ReportIDSHTPReportProductIDResponse
		_, err := b.waitForPacketType(ChannelCONTROL, &reportID, nil)
		if err != nil {
			b.debug("Error waiting for ID response packet:", err)
			return false, err
		}

		sensorID, err := b.parseSensorID()
		if err != nil {
			b.debug("Error parsing sensor ID:", err)
			return false, err
		}
		if sensorID != 0 {
			b.idRead = true
			return true, nil
		}
		b.debug("Packet didn't have sensor ID report, trying again")
	}
	// unreachable, but for completeness
	// return false
}

// parseSensorID parses the sensor ID from the data buffer.
//
// Returns:
//
// The sensor ID as an integer, or an error if the data buffer is too short or the report ID is incorrect.
func (b *BNO08X) parseSensorID() (int, error) {
	if b.dataBuffer[4] != ReportIDSHTPReportProductIDResponse {
		return 0, nil
	}

	// Check if the data buffer is long enough to read the expected bytes
	if len(b.dataBuffer) < 16 {
		return 0, errors.New("data buffer too short to read sensor ID")
	}

	// Parse the sensor ID from the data buffer
	swMajor, _ := b.getData(2, 1)
	swMinor, _ := b.getData(3, 1)
	swPatch, _ := b.getData(12, 2)
	swPartNumber, _ := b.getData(4, 4)
	swBuildNumber, _ := b.getData(8, 4)

	b.debug("\n** Sensor ID Report **")
	b.debug(fmt.Sprintf("*** Part Number: %d", swPartNumber))
	b.debug(fmt.Sprintf("*** Software Version: %d.%d.%d", swMajor, swMinor, swPatch))
	b.debug(fmt.Sprintf(" Build: %d", swBuildNumber))

	// TODO: this is only one of the numbers!
	return swPartNumber, nil
}

// getData retrieves data from the data buffer at the specified index and number of bytes.
//
// Parameters:
//
//  index: The index in the data buffer to start reading from (not including header).
//  numberBytes: The number of bytes to read from the data buffer.
//
// Returns:
//
// The integer value read from the data buffer, or an error if the buffer is too short.
func (b *BNO08X) getData(index, numberBytes int) (int, error) {
	// index is not including header, so add 4 into data buffer
	dataIndex := index + 4

	// Check if the data buffer is long enough to read the requested number of bytes
	if len(b.dataBuffer) < dataIndex+numberBytes {
		return 0, ErrBufferTooShort
	}

	switch numberBytes {
	case 1: // uint8
		return int(b.dataBuffer[dataIndex]), nil
	case 2: // uint16, little-endian
		return int(binary.LittleEndian.Uint16(b.dataBuffer[dataIndex:])), nil
	case 4: // uint32, little-endian
		return int(binary.LittleEndian.Uint32(b.dataBuffer[dataIndex:])), nil
	}
	return 0, fmt.Errorf("unsupported number of bytes: %d", numberBytes)
}

// waitForPacketType waits for a packet of a specific type on a given channel, optionally filtering by report ID.
//
// Parameters:
//
//  channelNumber: The channel number to wait for.
//  reportID: An optional pointer to a report ID to filter packets by.
//  timeout: An optional pointer to a duration to wait before timing out.
//
// Returns:
//
//  A pointer to the packet if found, or an error if the timeout is reached or an error occurs.
func (b *BNO08X) waitForPacketType(channelNumber uint8, reportID *uint8, timeout *time.Duration) (*packet, error) {
	startTime := time.Now()

	// Check if reportID is provided, and prepare a debug message accordingly
	reportIDStr := ""
	if reportID != nil {
		reportIDStr = fmt.Sprintf(" with report id 0x%X", *reportID)
	}
	b.debug("** Waiting for packet on channel", channelNumber, reportIDStr)

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
			b.debug("passing packet to handler for de-slicing")
			if err := b.handlePacket(newPacket); err != nil {
				return nil, err
			}
		}
	}
	return nil, fmt.Errorf("timed out waiting for a packet on channel %d", channelNumber)
}

// waitForPacket waits for a packet to be available from the packet reader within the specified timeout.
//
// Parameters:
//
//  timeout: The maximum duration to wait for a packet.
//
// Returns:
//
//  A pointer to the packet if available, or an error if the timeout is reached or an error occurs.
func (b *BNO08X) waitForPacket(timeout time.Duration) (*packet, error) {
	startTime := time.Now()
	for time.Since(startTime) < timeout {
		// Check if data is ready to be read
		if !b.packetReader.IsDataReady() {
			continue
		}

		// Read the packet from the packet reader
		newPacket, err := b.packetReader.ReadPacket()
		if err != nil {
			return nil, err
		}
		return newPacket, nil
	}
	return nil, ErrPacketTimeout
}

// handlePacket processes a packet by separating it into individual reports and processing each report.
//
// Parameters:
//
//  packet: A pointer to the packet to be processed.
//
// Returns:
//
//  An error if the packet cannot be processed, otherwise nil.
func (b *BNO08X) handlePacket(packet *packet) error {
	// Split out reports first
	if err := separateBatch(packet, &b.packetSlices); err != nil {
		fmt.Println(packet)
		return err
	}

	// Process each report in the packet slices
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
//  report: A pointer to the report to be processed.
//
// Returns:
//
//  An error if the report cannot be processed, otherwise nil.
func (b *BNO08X) processReport(report *report) error {
	// Check if the report is nil
	if report == nil {
		return ErrNilReport
	}

	if isControlReport(report.ID) {
		return b.handleControlReport(report)
	}

	b.debug("\tProcessing report:", Reports[report.ID])
	if b.debugFlag {
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
			return fmt.Errorf("failed to parse stability classifier report: %w", err)
		}

		// Update the stability classification in the BNO08X instance
		b.stabilityClassification = &stabilityReport.StabilityClassification
		return nil
	case ReportIDActivityClassifier:
		// Parse the activity classifier report
		activityReport, err := newActivityClassifierReport(report)
		if err != nil {
			return fmt.Errorf("failed to parse activity classifier report: %w", err)
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
			return fmt.Errorf("failed to parse geomagnetic rotation vector report: %w", err)
		}

		// Update the geomagnetic rotation vector readings in the BNO08X instance
		b.geomagneticRotationVector = &geomagneticReport.Results
	case ReportIDGameRotationVector:
		// Parse the game rotation vector report
		gameReport, err := newFourDimensionalReport(report)
		if err != nil {
			return fmt.Errorf("failed to parse game rotation vector report: %w", err)
		}

		// Update the game rotation vector readings in the BNO08X instance
		b.gameRotationVector = &gameReport.Results
	}

	// If we reach here, the report was processed successfully
	return nil
}

// handleControlReport processes control reports and updates the BNO08X state accordingly.
//
// Parameters:
//
//  report: A pointer to the report containing control data.
//
// Returns:
//
//  An error if the control report cannot be processed, otherwise nil.
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
		b.debug(fmt.Sprintf(
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

		// Check if the feature report ID is in the InitialReports map
		// featureReportID := getFeatureReport.ReportID
	case ReportIDCommandResponse:
		return b.handleCommandResponse(report)
	}
	return nil
}

// handleCommandResponse processes the command response report and updates the BNO08X state accordingly.
//
// Parameters:
//
//  report: A pointer to the report containing command response data.
//
// Returns:
//
//  An error if the command response cannot be processed, otherwise nil.
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

// processAvailablePackets processes all available packets from the packet reader, handling each packet until the maximum number of packets is reached.
//
// Parameters:
//
//  maxPackets: An optional pointer to an integer specifying the maximum number of packets to process. If nil, all available packets will be processed.
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
			b.debug("Error handling packet:", err)
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

@property
func steps(self) -> Optional[int]:
"""The number of steps detected since the sensor was initialized"""
self._process_available_packets()
try:
return self._readings[BnoReportStepCounter]
except KeyError:
raise RuntimeError("No steps report found, is it enabled?") from None

@property
func linear_acceleration(self) -> Optional[tuple[float, float, float]]:
"""A tuple representing the current linear acceleration values on the X, Y, and Z
axes in meters per second squared"""
self._process_available_packets()
try:
return self._readings[BnoReportLinearAcceleration]
except KeyError:
raise RuntimeError("No lin. accel report found, is it enabled?") from None

@property
func acceleration(self) -> Optional[tuple[float, float, float]]:
"""A tuple representing the acceleration measurements on the X, Y, and Z
axes in meters per second squared"""
self._process_available_packets()
try:
return self._readings[BnoReportAccelerometer]
except KeyError:
raise RuntimeError("No accel report found, is it enabled?") from None

@property
func gravity(self) -> Optional[tuple[float, float, float]]:
"""A tuple representing the gravity vector in the X, Y, and Z components
axes in meters per second squared"""
self._process_available_packets()
try:
return self._readings[BnoReportGravity]
except KeyError:
raise RuntimeError("No gravity report found, is it enabled?") from None

@property
func gyro(self) -> Optional[tuple[float, float, float]]:
"""A tuple representing Gyro's rotation measurements on the X, Y, and Z
axes in radians per second"""
self._process_available_packets()
try:
return self._readings[BnoReportGyroscope]
except KeyError:
raise RuntimeError("No gyro report found, is it enabled?") from None

@property
func shake(self) -> Optional[bool]:
"""True if a shake was detected on any axis since the last time it was checked

This property has a "latching" behavior where once a shake is detected, it will stay in a
"shaken" state until the value is read.This prevents missing shake events but means that
this property is not guaranteed to reflect the shake state at the moment it is read
"""
self._process_available_packets()
try:
shake_detected = self._readings[BnoReportShakeDetector]
// clear on read
if shake_detected:
self._readings[BnoReportShakeDetector] = False
return shake_detected
except KeyError:
raise RuntimeError("No shake report found, is it enabled?") from None

@property
func stability_classification(self) -> Optional[str]:
"""Returns the sensor's assessment of it's current stability, one of:

* "Unknown" - The sensor is unable to classify the current stability
* "On Table" - The sensor is at rest on a stable surface with very little vibration
* "Stationary" -  The sensor’s motion is below the stable threshold but\
the stable duration requirement has not been met.This output is only available when\
gyro calibration is enabled
* "Stable" - The sensor’s motion has met the stable threshold and duration requirements.
* "In motion" - The sensor is moving.

"""
self._process_available_packets()
try:
stability_classification = self._readings[BnoReportStabilityClassifier]
return stability_classification
except KeyError:
raise RuntimeError("No stability classification report found, is it enabled?") from None

@property
func activity_classification(self) -> Optional[dict]:
"""Returns the sensor's assessment of the activity that is creating the motions\
that it is sensing, one of:

* "Unknown"
* "In-Vehicle"
* "On-Bicycle"
* "On-Foot"
* "Still"
* "Tilting"
* "Walking"
* "Running"
* "On Stairs"

"""
self._process_available_packets()
try:
activity_classification = self._readings[BnoReportActivityClassifier]
return activity_classification
except KeyError:
raise RuntimeError("No activity classification report found, is it enabled?") from None

@property
func raw_acceleration(self) -> Optional[tuple[int, int, int]]:
"""Returns the sensor's raw, unscaled value from the accelerometer registers"""
self._process_available_packets()
try:
raw_acceleration = self._readings[BnoReportRawAccelerometer]
return raw_acceleration
except KeyError:
raise RuntimeError("No raw acceleration report found, is it enabled?") from None

@property
func raw_gyro(self) -> Optional[tuple[int, int, int]]:
"""Returns the sensor's raw, unscaled value from the gyro registers"""
self._process_available_packets()
try:
raw_gyro = self._readings[BnoReportRawGyroscope]
return raw_gyro
except KeyError:
raise RuntimeError("No raw gyro report found, is it enabled?") from None

@property
func raw_magnetic(self) -> Optional[tuple[int, int, int]]:
"""Returns the sensor's raw, unscaled value from the magnetometer registers"""
self._process_available_packets()
try:
raw_magnetic = self._readings[BnoReportRawMagnetometer]
return raw_magnetic
except KeyError:
raise RuntimeError("No raw magnetic report found, is it enabled?") from None

func begin_calibration(self) -> None:
"""Begin the sensor's self-calibration routine"""
// start calibration for accel, gyro, and mag
self._send_me_command(
[
1, // calibrate accel
1, // calibrate gyro
1, // calibrate mag
MECalibrationConfig,
0, // calibrate planar acceleration
0, // 'on_table' calibration
0, // reserved
0, // reserved
0,  // reserved
]
)
self._calibration_complete = False

@property
func calibration_status(self) -> int:
"""Get the status of the self-calibration"""
self._send_me_command(
[
0, // calibrate accel
0, // calibrate gyro
0, // calibrate mag
MEGetCalibration,
0, // calibrate planar acceleration
0, // 'on_table' calibration
0, // reserved
0, // reserved
0, // reserved
]
)
return self._magnetometer_accuracy

func _send_me_command(self, subcommand_params: Optional[list[int]]) -> None:
start_time = time.monotonic()
local_buffer = self._command_buffer
_insert_command_request_report(
MECalibrate,
self._command_buffer, // should use self._data_buffer :\ but send_packet don't
self._get_report_seq_id(CommandRequest),
subcommand_params,
)
self._send_packet(BnoChannelCONTROL, local_buffer)
self._increment_report_seq(CommandRequest)
while _elapsed(start_time) < DefaultTimeout:
self._process_available_packets()
if self._me_calibration_started_at > start_time:
break

func save_calibration_data(self) -> None:
"""Save the self-calibration data"""
// send a DCD save command
start_time = time.monotonic()
local_buffer = bytearray(12)
_insert_command_request_report(
SaveDCD,
local_buffer, // should use self._data_buffer :\ but send_packet don't
self._get_report_seq_id(CommandRequest),
)
self._send_packet(BnoChannelCONTROL, local_buffer)
self._increment_report_seq(CommandRequest)
while _elapsed(start_time) < DefaultTimeout:
self._process_available_packets()
if self._dcd_saved_at > start_time:
return
raise RuntimeError("Could not save calibration data")

// ############### private/helper methods ###############
// // decorator?
func _process_available_packets(
	self,
	max_packets: Optional[int] = None,
) -> None:
processed_count = 0
while self._data_ready:
if max_packets and processed_count > max_packets:
return
// print("reading a packet")
try:
new_packet = self._read_packet()
except PacketError:
continue
self._handle_packet(new_packet)
processed_count += 1
self._dbg("")
// print("Processed", processed_count, "packets")
self._dbg("")
self._dbg("")
self._dbg(" ** DONE! **")

// update the cached sequence number so we know what to increment from
// TODO: this is wrong there should be one per channel per direction
// and apparently per report as well
func _update_sequence_number(self, new_packet: Packet) -> None:
channel = new_packet.channel_number
seq = new_packet.header.sequence_number
self._sequence_number[channel] = seq

// TODO: add docs for available features
// TODO2: I think this should call an fn that imports all the bits for the given feature
// so we're not carrying around  stuff for extra features
func enable_feature(self, feature_id: int) -> None:
"""Used to enable a given feature of the BNO08x"""
self._dbg("\n********** Enabling feature id:", feature_id, "**********")

if feature_id == BnoReportActivityClassifier:
set_feature_report = self._get_feature_enable_report(
feature_id, sensor_specific_config = EnabledActivities
) else:
set_feature_report = self._get_feature_enable_report(feature_id)

feature_dependency = RawReports.get(feature_id, None)
// if the feature was enabled it will have a key in the readings dict
if feature_dependency and feature_dependency not in self._readings:
self._dbg("Enabling feature depencency:", feature_dependency)
self.enable_feature(feature_dependency)

self._dbg("Enabling", feature_id)
self._send_packet(BnoChannelCONTROL, set_feature_report)

start_time = time.monotonic()  // 1

while _elapsed(start_time) < FeatureEnableTimeout:
self._process_available_packets(max_packets = 10)
if feature_id in self._readings:
return
raise RuntimeError("Was not able to enable feature", feature_id)

func _increment_report_seq(self, report_id: int) -> None:
current = self._two_ended_sequence_numbers.get(report_id, 0)
self._two_ended_sequence_numbers[report_id] = (current + 1) % 256

func _get_report_seq_id(self, report_id: int) -> int:
return self._two_ended_sequence_numbers.get(report_id, 0)
