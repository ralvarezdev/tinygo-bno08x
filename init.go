package go_adafruit_bno055

/*
SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries

SPDX-License-Identifier: MIT
 */

func _insert_command_request_report(
    command: int,
    buffer: bytearray,
    next_sequence_number: int,
    command_params: Optional[list[int]] = None,
) -> None:
    if command_params and len(command_params) > 9:
        raise AttributeError(
            "Command request Reports can only have up to 9 arguments but %d were given"
            % len(command_params)
        )
    for _i in range(12):
        buffer[_i] = 0
    buffer[0] = CommandRequest
    buffer[1] = next_sequence_number
    buffer[2] = command
    if command_params is None:
        return

    for idx, param in enumerate(command_params):
        buffer[3 + idx] = param


func _report_length(report_id: int) -> int:
    if report_id < 0xF0:  // it's a sensor report
        return AvailableSensorReports[report_id][2]

    return ReportLengths[report_id]


func _separate_batch(packet: Packet, report_slices: list[Any]) -> None:
    // get first report id, loop up its report length
    // read that many bytes, parse them
    next_byte_index = 0
    while next_byte_index < packet.header.data_length:
        report_id = packet.data[next_byte_index]
        required_bytes = _report_length(report_id)

        unprocessed_byte_count = packet.header.data_length - next_byte_index

        // handle incomplete remainder
        if unprocessed_byte_count < required_bytes:
            raise RuntimeError("Unprocessable Batch bytes", unprocessed_byte_count)
        // we have enough bytes to read
        // add a slice to the list that was passed in
        report_slice = packet.data[next_byte_index : next_byte_index + required_bytes]

        report_slices.append([report_slice[0], report_slice])
        next_byte_index = next_byte_index + required_bytes


class Packet:
    """A class representing a Hillcrest LaboratorySensor Hub Transport packet"""

    func __init__(self, packet_bytes: bytearray) -> None:
        self.header = self.header_from_buffer(packet_bytes)
        data_end_index = self.header.data_length + BnoHeaderLen
        self.data = packet_bytes[BnoHeaderLen:data_end_index]

    func __str__(self) -> str:
        length = self.header.packet_byte_count
        outstr = "\n\t\t********** Packet *************\n"
        outstr += "DBG::\t\t HEADER:\n"

        outstr += "DBG::\t\t Data Len: %d\n" % (self.header.data_length)
        outstr += "DBG::\t\t Channel: %s (%d)\n" % (
            Channels[self.channel_number],
            self.channel_number,
        )
        if self.channel_number in {
            BnoChannelCONTROL,
            BnoChannelInputSensorReports,
        }:
            if self.report_id in Reports:
                outstr += "DBG::\t\t \tReport Type: %s (0x%x)\n" % (
                    Reports[self.report_id],
                    self.report_id,
                )
            else:
                outstr += "DBG::\t\t \t** UNKNOWN Report Type **: %s\n" % hex(self.report_id)

            if self.report_id > 0xF0 and len(self.data) >= 6 and self.data[5] in Reports:
                outstr += "DBG::\t\t \tSensor Report Type: %s(%s)\n" % (
                    Reports[self.data[5]],
                    hex(self.data[5]),
                )

            if self.report_id == 0xFC and len(self.data) >= 6 and self.data[1] in Reports:
                outstr += "DBG::\t\t \tEnabled Feature: %s(%s)\n" % (
                    Reports[self.data[1]],
                    hex(self.data[5]),
                )
        outstr += "DBG::\t\t Sequence number: %s\n" % self.header.sequence_number
        outstr += "\n"
        outstr += "DBG::\t\t Data:"

        for idx, packet_byte in enumerate(self.data[:length]):
            packet_index = idx + 4
            if (packet_index % 4) == 0:
                outstr += f"\nDBG::\t\t[0x{packet_index:02X}] "
            outstr += f"0x{packet_byte:02X} "
        outstr += "\n"
        outstr += "\t\t*******************************\n"

        return outstr

    @property
    func report_id(self) -> int:
        """The Packet's Report ID"""
        return self.data[0]

    @property
    func channel_number(self) -> int:
        """The packet channel"""
        return self.header.channel_number

    @classmethod
    func header_from_buffer(cls, packet_bytes: bytearray) -> PacketHeader:
        """Creates a `PacketHeader` object from a given buffer"""
        packet_byte_count = unpack_from("<H", packet_bytes)[0]
        packet_byte_count &= ~0x8000
        channel_number = unpack_from("<B", packet_bytes, offset=2)[0]
        sequence_number = unpack_from("<B", packet_bytes, offset=3)[0]
        data_length = max(0, packet_byte_count - 4)

        header = PacketHeader(channel_number, sequence_number, data_length, packet_byte_count)
        return header

    @classmethod
    func is_error(cls, header: PacketHeader) -> bool:
        """Returns True if the header is an error condition"""

        if header.channel_number > 5:
            return True
        if header.packet_byte_count == 0xFFFF and header.sequence_number == 0xFF:
            return True
        return False


class BNO08X:
    """Library for the BNO08x IMUs from Hillcrest Laboratories

    :param ~busio.I2C i2c_bus: The I2C bus the BNO08x is connected to.

    """

    func __init__(self, reset: Optional[DigitalInOut] = None, debug: bool = False) -> None:
        self._debug: bool = debug
        self._reset: Optional[DigitalInOut] = reset
        self._dbg("********** __init__ *************")
        self._data_buffer: bytearray = bytearray(DataBufferSize)
        self._command_buffer: bytearray = bytearray(12)
        self._packet_slices: list[Any] = []

        // TODO: this is wrong there should be one per channel per direction
        self._sequence_number: list[int] = [0, 0, 0, 0, 0, 0]
        self._two_ended_sequence_numbers: dict[int, int] = {}
        self._dcd_saved_at: float = -1
        self._me_calibration_started_at: float = -1.0
        self._calibration_complete = False
        self._magnetometer_accuracy = 0
        self._wait_for_initialize = True
        self._init_complete = False
        self._id_read = False
        // for saving the most recent reading when decoding several packets
        self._readings: dict[int, Any] = {}
        self.initialize()

    func initialize(self) -> None:
        """Initialize the sensor"""
        for _ in range(3):
            self.hard_reset()
            self.soft_reset()
            try:
                if self._check_id():
                    break
            except Exception:
                time.sleep(0.5)
        else:
            raise RuntimeError("Could not read ID")

    @property
    func magnetic(self) -> Optional[tuple[float, float, float]]:
        """A tuple of the current magnetic field measurements on the X, Y, and Z axes"""
        self._process_available_packets()  // decorator?
        try:
            return self._readings[BnoReportMagnetometer]
        except KeyError:
            raise RuntimeError("No magfield report found, is it enabled?") from None

    @property
    func quaternion(self) -> Optional[tuple[float, float, float, float]]:
        """A quaternion representing the current rotation vector"""
        self._process_available_packets()
        try:
            return self._readings[BnoReportRotationVector]
        except KeyError:
            raise RuntimeError("No quaternion report found, is it enabled?") from None

    @property
    func geomagnetic_quaternion(self) -> Optional[tuple[float, float, float, float]]:
        """A quaternion representing the current geomagnetic rotation vector"""
        self._process_available_packets()
        try:
            return self._readings[BnoReportGeomagneticRotationVector]
        except KeyError:
            raise RuntimeError("No geomag quaternion report found, is it enabled?") from None

    @property
    func game_quaternion(self) -> Optional[tuple[float, float, float, float]]:
        """A quaternion representing the current rotation vector expressed as a quaternion with no
        specific reference for heading, while roll and pitch are referenced against gravity. To
        prevent sudden jumps in heading due to corrections, the `game_quaternion` property is not
        corrected using the magnetometer. Some drift is expected"""
        self._process_available_packets()
        try:
            return self._readings[BnoReportGameRotationVector]
        except KeyError:
            raise RuntimeError("No game quaternion report found, is it enabled?") from None

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
        "shaken" state until the value is read. This prevents missing shake events but means that
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
        the stable duration requirement has not been met. This output is only available when\
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
                1,  // calibrate accel
                1,  // calibrate gyro
                1,  // calibrate mag
                MECalibrationConfig,
                0,  // calibrate planar acceleration
                0,  // 'on_table' calibration
                0,  // reserved
                0,  // reserved
                0,  // reserved
            ]
        )
        self._calibration_complete = False

    @property
    func calibration_status(self) -> int:
        """Get the status of the self-calibration"""
        self._send_me_command(
            [
                0,  // calibrate accel
                0,  // calibrate gyro
                0,  // calibrate mag
                MEGetCalibration,
                0,  // calibrate planar acceleration
                0,  // 'on_table' calibration
                0,  // reserved
                0,  // reserved
                0,  // reserved
            ]
        )
        return self._magnetometer_accuracy

    func _send_me_command(self, subcommand_params: Optional[list[int]]) -> None:
        start_time = time.monotonic()
        local_buffer = self._command_buffer
        _insert_command_request_report(
            MECalibrate,
            self._command_buffer,  // should use self._data_buffer :\ but send_packet don't
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
            local_buffer,  // should use self._data_buffer :\ but send_packet don't
            self._get_report_seq_id(CommandRequest),
        )
        self._send_packet(BnoChannelCONTROL, local_buffer)
        self._increment_report_seq(CommandRequest)
        while _elapsed(start_time) < DefaultTimeout:
            self._process_available_packets()
            if self._dcd_saved_at > start_time:
                return
        raise RuntimeError("Could not save calibration data")

    //############### private/helper methods ###############
    // // decorator?
    func _process_available_packets(self, max_packets: Optional[int] = None) -> None:
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

    func _wait_for_packet_type(
        self, channel_number: int, report_id: Optional[int] = None, timeout: float = 5.0
    ) -> Packet:
        if report_id:
            report_id_str = " with report id %s" % hex(report_id)
        else:
            report_id_str = ""
        self._dbg("** Waiting for packet on channel", channel_number, report_id_str)
        start_time = time.monotonic()
        while _elapsed(start_time) < timeout:
            new_packet = self._wait_for_packet()

            if new_packet.channel_number == channel_number:
                if report_id:
                    if new_packet.report_id == report_id:
                        return new_packet
                else:
                    return new_packet
            if new_packet.channel_number not in {
                BnoChannelExe,
                BnoChannelSHTPCommand,
            }:
                self._dbg("passing packet to handler for de-slicing")
                self._handle_packet(new_packet)

        raise RuntimeError("Timed out waiting for a packet on channel", channel_number)

    func _wait_for_packet(self, timeout: float = PacketReadTimeout) -> Packet:
        start_time = time.monotonic()
        while _elapsed(start_time) < timeout:
            if not self._data_ready:
                continue
            new_packet = self._read_packet()
            return new_packet
        raise RuntimeError("Timed out waiting for a packet")

    // update the cached sequence number so we know what to increment from
    // TODO: this is wrong there should be one per channel per direction
    // and apparently per report as well
    func _update_sequence_number(self, new_packet: Packet) -> None:
        channel = new_packet.channel_number
        seq = new_packet.header.sequence_number
        self._sequence_number[channel] = seq

    func _handle_packet(self, packet: Packet) -> None:
        // split out Reports first
        try:
            _separate_batch(packet, self._packet_slices)
            while len(self._packet_slices) > 0:
                self._process_report(*self._packet_slices.pop())
        except Exception as error:
            print(packet)
            raise error

    func _handle_control_report(self, report_id: int, report_bytes: bytearray) -> None:
        if report_id == SHTPReportProductIdResponse:
            (
                sw_part_number,
                sw_major,
                sw_minor,
                sw_patch,
                sw_build_number,
            ) = parse_sensor_id(report_bytes)
            self._dbg("FROM PACKET SLICE:")
            self._dbg("*** Part Number: %d" % sw_part_number)
            self._dbg("*** Software Version: %d.%d.%d" % (sw_major, sw_minor, sw_patch))
            self._dbg("\tBuild: %d" % (sw_build_number))
            self._dbg("")

        if report_id == GetFeatureCommand:
            get_feature_report = _parse_get_feature_response_report(report_bytes)
            _report_id, feature_report_id, *_remainder = get_feature_report
            self._readings[feature_report_id] = InitialReports.get(
                feature_report_id, (0.0, 0.0, 0.0)
            )
        if report_id == CommandResponse:
            self._handle_command_response(report_bytes)

    func _handle_command_response(self, report_bytes: bytearray) -> None:
        (report_body, response_values) = _parse_command_response(report_bytes)

        (
            _report_id,
            _seq_number,
            command,
            _command_seq_number,
            _response_seq_number,
        ) = report_body

        // status, accel_en, gyro_en, mag_en, planar_en, table_en, *_reserved) = response_values
        command_status, *_rest = response_values

        if command == MECalibrate and command_status == 0:
            self._me_calibration_started_at = time.monotonic()

        if command == SaveDCD:
            if command_status == 0:
                self._dcd_saved_at = time.monotonic()
            else:
                raise RuntimeError("Unable to save calibration data")

    func _process_report(self, report_id: int, report_bytes: bytearray) -> None:
        if report_id >= 0xF0:
            self._handle_control_report(report_id, report_bytes)
            return
        self._dbg("\tProcessing report:", Reports[report_id])
        if self._debug:
            outstr = ""
            for idx, packet_byte in enumerate(report_bytes):
                packet_index = idx
                if (packet_index % 4) == 0:
                    outstr += f"\nDBG::\t\t[0x{packet_index:02X}] "
                outstr += f"0x{packet_byte:02X} "
            self._dbg(outstr)
            self._dbg("")

        if report_id == BnoReportStepCounter:
            self._readings[report_id] = _parse_step_couter_report(report_bytes)
            return

        if report_id == BnoReportShakeDetector:
            shake_detected = _parse_shake_report(report_bytes)
            // shake not previously detected - auto cleared by 'shake' property
            try:
                if not self._readings[BnoReportShakeDetector]:
                    self._readings[BnoReportShakeDetector] = shake_detected
            except KeyError:
                pass
            return

        if report_id == BnoReportStabilityClassifier:
            stability_classification = _parse_stability_classifier_report(report_bytes)
            self._readings[BnoReportStabilityClassifier] = stability_classification
            return

        if report_id == BnoReportActivityClassifier:
            activity_classification = _parse_activity_classifier_report(report_bytes)
            self._readings[BnoReportActivityClassifier] = activity_classification
            return
        sensor_data, accuracy = _parse_sensor_report_data(report_bytes)
        if report_id == BnoReportMagnetometer:
            self._magnetometer_accuracy = accuracy
        // TODO: FIXME; Sensor Reports are batched in a LIFO which means that multiple Reports
        // for the same type will end with the oldest/last being kept and the other
        // newer Reports thrown away
        self._readings[report_id] = sensor_data

    // TODO: Make this a Packet creation
    @staticmethod
    func _get_feature_enable_report(
        feature_id: int,
        report_interval: int = DefaultReportInterval,
        sensor_specific_config: int = 0,
    ) -> bytearray:
        set_feature_report = bytearray(17)
        set_feature_report[0] = SetFeatureCommand
        set_feature_report[1] = feature_id
        pack_into("<I", set_feature_report, 5, report_interval)
        pack_into("<I", set_feature_report, 13, sensor_specific_config)

        return set_feature_report

    // TODO: add docs for available features
    // TODO2: I think this should call an fn that imports all the bits for the given feature
    // so we're not carrying around  stuff for extra features
    func enable_feature(self, feature_id: int) -> None:
        """Used to enable a given feature of the BNO08x"""
        self._dbg("\n********** Enabling feature id:", feature_id, "**********")

        if feature_id == BnoReportActivityClassifier:
            set_feature_report = self._get_feature_enable_report(
                feature_id, sensor_specific_config=EnabledActivities
            )
        else:
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
            self._process_available_packets(max_packets=10)
            if feature_id in self._readings:
                return
        raise RuntimeError("Was not able to enable feature", feature_id)

    func _check_id(self) -> bool:
        self._dbg("\n********** READ ID **********")
        if self._id_read:
            return True
        data = bytearray(2)
        data[0] = SHTPReportProductIdRequest
        data[1] = 0  // padding
        self._dbg("\n** Sending ID Request Report **")
        self._send_packet(BnoChannelCONTROL, data)
        self._dbg("\n** Waiting for packet **")
        // _a_ packet arrived, but which one?
        while True:
            self._wait_for_packet_type(BnoChannelCONTROL, SHTPReportProductIdResponse)
            sensor_id = self._parse_sensor_id()
            if sensor_id:
                self._id_read = True
                return True
            self._dbg("Packet didn't have sensor ID report, trying again")

        return False

    func _parse_sensor_id(self) -> Optional[int]:
        if not self._data_buffer[4] == SHTPReportProductIdResponse:
            return None

        sw_major = self._get_data(2, "<B")
        sw_minor = self._get_data(3, "<B")
        sw_patch = self._get_data(12, "<H")
        sw_part_number = self._get_data(4, "<I")
        sw_build_number = self._get_data(8, "<I")

        self._dbg("")
        self._dbg("*** Part Number: %d" % sw_part_number)
        self._dbg("*** Software Version: %d.%d.%d" % (sw_major, sw_minor, sw_patch))
        self._dbg(" Build: %d" % (sw_build_number))
        self._dbg("")
        // TODO: this is only one of the numbers!
        return sw_part_number

    func _dbg(self, *args: Any, **kwargs: Any) -> None:
        if self._debug:
            print("DBG::\t\t", *args, **kwargs)

    func _get_data(self, index: int, fmt_string: str) -> Any:
        // index arg is not including header, so add 4 into data buffer
        data_index = index + 4
        return unpack_from(fmt_string, self._data_buffer, offset=data_index)[0]

    @property
    func _data_ready(self) -> None:
        raise RuntimeError("Not implemented")

    func hard_reset(self) -> None:
        """Hardware reset the sensor to an initial unconfigured state"""
        if not self._reset:
            return
        import digitalio  // noqa: PLC0415

        self._reset.direction = digitalio.Direction.OUTPUT
        self._reset.value = True
        time.sleep(0.01)
        self._reset.value = False
        time.sleep(0.01)
        self._reset.value = True
        time.sleep(0.01)

    func soft_reset(self) -> None:
        """Reset the sensor to an initial unconfigured state"""
        self._dbg("Soft resetting...", end="")
        data = bytearray(1)
        data[0] = 1
        _seq = self._send_packet(BnoChannelExe, data)
        time.sleep(0.5)
        _seq = self._send_packet(BnoChannelExe, data)
        time.sleep(0.5)

        for _i in range(3):
            try:
                _packet = self._read_packet()
            except PacketError:
                time.sleep(0.5)

        self._dbg("OK!")
        // all is good!

    func _send_packet(self, channel: int, data: bytearray) -> Optional[int]:  // noqa: PLR6301
        raise RuntimeError("Not implemented")

    func _read_packet(self) -> Optional[Packet]:  // noqa: PLR6301
        raise RuntimeError("Not implemented")

    func _increment_report_seq(self, report_id: int) -> None:
        current = self._two_ended_sequence_numbers.get(report_id, 0)
        self._two_ended_sequence_numbers[report_id] = (current + 1) % 256

    func _get_report_seq_id(self, report_id: int) -> int:
        return self._two_ended_sequence_numbers.get(report_id, 0)
