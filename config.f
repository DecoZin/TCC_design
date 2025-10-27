// Files

lint.vlt

// User commands
--timing


// Strict Warnings
-Wall
// Don't exit on warnings
-Wno-fatal

// Multithreading
-j 0

// enable SVA
--assert

// dump as FST(Waveform compressed(VCD))
--trace-fst
--trace-max-array 512

//dump structs as Human-Readable format
--trace-structs

// All explicit Xs are replaced by a constant value determined at runtime
--x-assign unique
// All variables are randomly initialized
--x-initial unique
