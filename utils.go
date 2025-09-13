//go:build tinygo && (rp2040 || rp2350)

package tinygo_bno08x

import (
	"time"

	"machine"

	tinygotypes "github.com/ralvarezdev/tinygo-types"
	tinygologger "github.com/ralvarezdev/tinygo-logger"
)

var (
	// hardwareResetStart is the initial message printed when performing a hardware reset
	hardwareResetStart = []byte("Hardware resetting...")

	// hardwareResetComplete is the message printed when a hardware reset is complete
	hardwareResetComplete = []byte("Hardware reset complete")

	// errorInAfterHardwareResetFn is the message printed when there is an error in the afterHardwareResetFn
	errorInAfterHardwareResetFn = []byte("Error in afterHardwareResetFn:")

	// softwareResetStart is the initial message printed when performing a software reset
	softwareResetStart = []byte("Software resetting...")

	// softwareResetComplete is the message printed when a software reset is complete
	softwareResetComplete = []byte("Software reset complete")

	// foundSHTPAdvertisementPacket is the message printed when an SHTP advertisement packet is found
	foundSHTPAdvertisementPacket = []byte("Found SHTP advertisement packet")

	// clearingPacketFromChannel is the prefix message printed when clearing a packet from a channel
	clearingPacketFromChannel = []byte("Clearing packet from channel")
)

// HardwareReset performs a hardware reset of the BNO08X sensor to an initial unconfigured state.
//
// Parameters:
//
// reset: The machine.Pin used to perform the hardware reset.
// logger: An optional Logger for logging debug information during the reset process.
func HardwareReset(resetPin machine.Pin, logger tinygologger.Logger) {
	if logger != nil {
		logger.InfoMessage(hardwareResetStart)
	}

	// Configure the reset pin as output
	resetPin.Configure(machine.PinConfig{Mode: machine.PinOutput})

	resetPin.High()
	time.Sleep(ResetPinDelay)

	resetPin.Low()
	time.Sleep(ResetPinDelay)

	resetPin.High()
	time.Sleep(ResetPinDelay)

	if logger != nil {
		logger.InfoMessage(hardwareResetComplete)
	}
}

// SoftwareResetForI2CAndSPIMode performs a software reset of the BNO08X sensor when operating in I2C or SPI mode.
//
// Parameters:
//
// packetWriter: The PacketWriter used to send the reset command.
// logger: An optional Logger for logging debug information during the reset process.
// waitForPacketFn: A function that waits for a packet to be available, with a specified timeout.
//
// Returns:
//
// An error if the packet writer or waitForPacket function is nil, if sending the reset command fails,
// or if there are issues while waiting for packets during the reset process.
func SoftwareResetForI2CAndSPIMode(packetWriter PacketWriter, logger tinygologger.Logger, waitForPacketFn func(time.Duration) (Packet, tinygotypes.ErrorCode)) tinygotypes.ErrorCode {
	// Check if the packet writer is nil
	if packetWriter == nil {
		return ErrorCodeBNO08XNilPacketWriter
	}

	// Check if the waitForPacket function is nil
	if waitForPacketFn == nil {
		return ErrorCodeBNO08XNilWaitForPacketFunction
	}


	// Log the start of the reset process
	if logger != nil {
		logger.InfoMessage(softwareResetStart)
	}

	// Send the reset command
	if _, err := packetWriter.SendPacket(ChannelExe, ExecCommandResetData); err != tinygotypes.ErrorCodeNil {
		return ErrorCodeBNO08XFailedToSendResetCommandRequestPacket
	}

	// Wait a bit for the reset to take effect
	time.Sleep(ResetCommandDelay)

	// Clear out any pending packets
	startTime := time.Now()
	for time.Since(startTime) < MaxClearPendingPacketsTimeout {
		packet, err := waitForPacketFn(WaitForPacketTimeout)
		if err == ErrorCodeBNO08XWaitingForPacketTimedOut {
			break
		}
		if err != tinygotypes.ErrorCodeNil {
			logger.WarningMessageWithErrorCode(errorWaitingForPacket, err, true)
			continue
		}

		// Log what we're clearing
		if logger != nil {
			if packet.ChannelNumber() == ChannelSHTPCommand && len(packet.Data) == AdvertisementPacketLength {
				logger.InfoMessage(foundSHTPAdvertisementPacket)
			} else {
				logger.AddMessageWithUint8(clearingPacketFromChannel, packet.ChannelNumber(), true, true, true)
				logger.Info()
			}
		}
	}

	if logger != nil {
		logger.InfoMessage(softwareResetComplete)
	}
	return tinygotypes.ErrorCodeNil
}

// SoftwareResetForUARTMode performs a software reset of the BNO08X sensor when operating in UART mode.
//
// Parameters:
//
// packetWriter: The PacketWriter used to send the reset command.
// logger: An optional Logger for logging debug information during the reset process.
// waitForPacketFn: A function that waits for a packet to be available, with a specified timeout.
//
// Returns:
//
// An error if the packet writer or waitForPacket function is nil, if sending the reset command fails,
// or if there are issues while waiting for packets during the reset process.
func SoftwareResetForUARTMode(packetWriter PacketWriter, logger tinygologger.Logger, waitForPacketFn func(time.Duration) (Packet, tinygotypes.ErrorCode)) tinygotypes.ErrorCode {
	// Check if the packet writer is nil
	if packetWriter == nil {
		return ErrorCodeBNO08XNilPacketWriter
	}

	// Check if the waitForPacket function is nil
	if waitForPacketFn == nil {
		return ErrorCodeBNO08XNilWaitForPacketFunction
	}

	// Log the start of the reset process
	if logger != nil {
		logger.InfoMessage(softwareResetStart)
	}

	// Clear out any pending packets
	startTime := time.Now()
	for time.Since(startTime) < MaxClearPendingPacketsTimeout {
		packet, err := waitForPacketFn(WaitForPacketTimeout)
		if err == ErrorCodeBNO08XWaitingForPacketTimedOut {
			break
		}
		if err != tinygotypes.ErrorCodeNil {
			logger.WarningMessageWithErrorCode(errorWaitingForPacket, err, true)
			continue
		}

		// Log what we're clearing
		if logger != nil {
			if packet.ChannelNumber() == ChannelSHTPCommand && len(packet.Data) == AdvertisementPacketLength {
				logger.InfoMessage(foundSHTPAdvertisementPacket)
			} else {
				logger.AddMessageWithUint8(clearingPacketFromChannel, packet.ChannelNumber(), true, true, true)
				logger.Info()
			}
		}
	}

	// Send the reset command
	if _, err := packetWriter.SendPacket(ChannelExe, ExecCommandResetData); err != tinygotypes.ErrorCodeNil {
		return ErrorCodeBNO08XFailedToSendResetCommandRequestPacket
	}

	// Wait a bit for the reset to take effect
	time.Sleep(ResetCommandDelay)

	if logger != nil {
		logger.InfoMessage(softwareResetComplete)
	}
	return tinygotypes.ErrorCodeNil
}