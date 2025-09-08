//go:build tinygo && (rp2040 || rp2350)

package tinygo_bno08x

import (
	"fmt"
	"strings"
)

type (
	// Debugger is an interface for debugging messages
	Debugger interface {
		Debug(args ...any)
	}

	// DefaultDebugger is a simple implementation of the Debugger interface
	DefaultDebugger struct{}
)

// NewDefaultDebugger creates a new DefaultDebugger instance
func NewDefaultDebugger() *DefaultDebugger {
	return &DefaultDebugger{}
}

// Debug function to print debug messages
//
// Parameters:
//
//	args: The arguments to print in the debug message
func (d *DefaultDebugger) Debug(args ...any) {
	var b strings.Builder

	b.WriteString(DebugHeader)
	b.WriteString(" ")
	for i, arg := range args {
		if i > 0 {
			b.WriteString("\n ")
		}
		b.WriteString(fmt.Sprintf("%v", arg))
	}
	fmt.Println(b.String())
}
