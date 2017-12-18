#ifndef TYPES
#define TYPES

#include <QString>

// Describes the type of element for a slot (input or output)
/*
enum ParameterType {
    Scalar, // One scalar value (floating point value)
    Vector, // A one-dimension matrix
    Matrix  // A 2-D (or more) matrix
};
//*/

/*
// Type that describes the common structure for a parameter slot
struct ParameterSlot {
    QString name;       // The name of the parameter
    ParameterType type; // The type of elements for this parameter slot
};


// Type that describes an input to a neural function
struct InputSlot : ParameterSlot {
    bool allowMultiple; // Whether this input slot can accept several parameters (ignored for output slots)
};

// Type that describes an output to a neural function
struct OutputSlot : ParameterSlot {
    // No difference for now, but it less awkward that having 'InputSlot' inherits from 'OutputSlot'
};
//*/

#endif // TYPES

