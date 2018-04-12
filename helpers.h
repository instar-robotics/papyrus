#ifndef HELPERS_H
#define HELPERS_H

#include "papyruswindow.h"
#include "slot.h"

#include <QApplication>
#include <QString>

/**
 * This file provides a set of helpers that are used throughout this project.
 */

// Define the maximum number of rows / columns a matrix can have (for the spinbox)
#define MAX_ROWS 100000
#define MAX_COLS 100000

// Define the minimum and maximum allowed weight (for the double spinbox)
#define MIN_WEIGHT -10000
#define MAX_WEIGHT 10000

PapyrusWindow *getMainWindow();

QString outputTypeToString(OutputType outputType);
OutputType stringToOutputType(QString str);

QString inputTypeToString(InputType inputType);
InputType stringToInputType(QString str);

bool canLink(OutputType from, InputType to);

#endif // HELPERS_H
