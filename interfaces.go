package tinygo_bno08x

type (
	// BNO08XService is an interface to wrap the BNO08X implementation methods.
	BNO08XService interface {
		GetAcceleration() *[3]float64
		GetEulerDegrees() *[3]float64
		Initialize() error
		HardwareReset()
		SoftwareReset() error
		Reset() error
	}
)
