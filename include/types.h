#ifndef TYPES_H
#define TYPES_H

/**
 * This is just a collection of types (no functions!) in order to simply the include process
 */

// Define the type that an @InputSlot accepts
enum InputType{
    SCALAR_SCALAR,
    SCALAR_MATRIX,
    MATRIX_MATRIX,
    SPARSE_MATRIX,
    STRING_INPUT
};

// Define the type that an @OutputSlot (hence a function, really) outputs
enum OutputType{
    SCALAR,
    MATRIX,
    STRING
};

// Define whether the time value is a frequency in Hz or a period in ms for a @Script
enum TimeUnit{
    HZ,
    MS
};

// Define the urgency of a message that will be shown on the status bar (and then its color)
enum MessageUrgency {
    MSG_INFO,
    MSG_WARNING,
    MSG_ERROR
};

#endif // TYPES_H
