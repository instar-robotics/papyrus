/*
  Copyright (C) INSTAR Robotics

  Author: Nicolas SCHOEMAEKER
 
  This file is part of papyrus <https://github.com/instar-robotics/papyrus>.
 
  papyrus is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  papyrus is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/

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
	STRING_INPUT
};

// Define the type that an @OutputSlot (hence a function, really) outputs
enum OutputType{
	INVALID_OUTPUT_TYPE,
	SCALAR,
	MATRIX,
	STRING
};

// Define the shape of a matrix-typed @DiagramBox
enum MatrixShape {
	INVALID_MATRIX_SHAPE,
	SHAPE_NONE,
	POINT,
	VECT,
	ROW_VECT,
	COL_VECT
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

// Define the different reasons why a Link can be invalid
enum LinkInvalidReason {
	INVALID_INVALID_REASON = 0, // MUST be zero (because it uses bitwise OR)
	TYPES_INCOMPATIBLE     = 0b1,
	SIZES_DONT_MATCH       = 0b10,
	SHAPE_MUST_BE_POINT    = 0b100,
	SHAPE_MUST_BE_VECT     = 0b1000,
	SHAPE_MUST_BE_ROW_VECT = 0b10000,
	SHAPE_MUST_BE_COL_VECT = 0b100000
};

// Defining bitwise operations for LinkInvalidReason because in C++11, enums are scoped
inline LinkInvalidReason operator|(LinkInvalidReason a, LinkInvalidReason b) {
	return static_cast<LinkInvalidReason>(static_cast<int>(a) | static_cast<int>(b));
}

inline LinkInvalidReason operator&(LinkInvalidReason a, LinkInvalidReason b) {
	return static_cast<LinkInvalidReason>(static_cast<int>(a) & static_cast<int>(b));
}

// Define the different reasons why a box can be invalid
enum BoxInvalidReason {
	INVALID_BOX_INVALID_REASON = 0, // MUST be zero (because it uses bitwise OR)
	INPUT_FULL                 = 0b1,
	BOX_MUST_BE_POINT          = 0b10,
	BOX_MUST_BE_VECT           = 0b100,
	BOX_MUST_BE_ROW_VECT       = 0b1000,
	BOX_MUST_BE_COL_VECT       = 0b10000,
};

// Defining bitwise operations for BoxInvalidReason because in C++11, enums are scoped
inline BoxInvalidReason operator|(BoxInvalidReason a, BoxInvalidReason b) {
	return static_cast<BoxInvalidReason>(static_cast<int>(a) | static_cast<int>(b));
}

inline BoxInvalidReason operator&(BoxInvalidReason a, BoxInvalidReason b) {
	return static_cast<BoxInvalidReason>(static_cast<int>(a) & static_cast<int>(b));
}

// Define the type of resizing for items we re-implement resizing
enum ResizeType {
	NO_RESIZE,
	RESIZE_TOP,
	RESIZE_RIGHT,
	RESIZE_BOTTOM,
	RESIZE_LEFT,
	RESIZE_TOP_LEFT,
	RESIZE_TOP_RIGHT,
	RESIZE_BOTTOM_RIGHT,
	RESIZE_BOTTOM_LEFT
};

// Define the different control that can be used with the mouse
enum MouseControl{
    LEFT_BUTTON,
    RIGHT_BUTTON,
    RIGHT_CTRL_BUTTON
};
#endif // TYPES_H
