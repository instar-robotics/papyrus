#ifndef TYPES_H
#define TYPES_H

/**
 * This is just a collection of types (no functions!) in order to simply the include process
 */

// Define the type that an @InputSlot accepts
enum InputType{
	INVALID_INPUT_TYPE,
	SCALAR_SCALAR,
	SCALAR_MATRIX,
	MATRIX_MATRIX,
	SPARSE_MATRIX,
	STRING_INPUT
};

// Define the type that an @OutputSlot (hence a function, really) outputs
enum OutputType{
	INVALID_OUTPUT_TYPE,
	SCALAR,
	MATRIX,
	STRING
};

// Define whether the time value is a frequency in Hz or a period in ms for a @Script
enum TimeUnit{
	INVALID_UNIT,
	HZ,
	MS
};

// Define the urgency of a message that will be shown on the status bar (and then its color)
enum MessageUrgency {
	INVALID_MESSAGE_URGENCY,
	MSG_INFO,
	MSG_WARNING,
	MSG_ERROR
};

// Define the type of connectivity for MATRIX_MATRIX links
enum Connectivity {
	INVALID_CONNECTIVITY,
	ONE_TO_ONE,
	ONE_TO_ALL,
	ONE_TO_NEI
};

// Define the type of visualization for data
enum VisualizationType {
	INVALID_VISUALIZATION_TYPE,
	BAR,        // scalar and vector
	GRAPH,      // scalar and vector
	IMAGE,      // matrix
	GRAYSCALE,  // matrix
	LANDSCAPE   // matrix
};

#endif // TYPES_H
