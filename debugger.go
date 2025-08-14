package go_adafruit_bno08x

import "fmt"

type (
	// Debugger is an interface for debugging messages
	Debugger interface {
		Debug(args ...any)
	}

	// DefaultDebugger is a simple implementation of the Debugger interface
	DefaultDebugger struct{}
)

// newDefaultDebugger creates a new DefaultDebugger instance
func newDefaultDebugger() *DefaultDebugger {
	return &DefaultDebugger{}
}

// Debug function to print debug messages
//
// Parameters:
//
//	args: The arguments to print in the debug message
func (d *DefaultDebugger) Debug(args ...any) {
	fmt.Print("DBG::\t\t")
	fmt.Print(args...)
	fmt.Println()
}
