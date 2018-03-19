#ifndef HELPERS_H
#define HELPERS_H

#include "papyruswindow.h"
#include "slot.h"

#include <QApplication>
#include <QString>

/**
 * This file provides a set of helpers that are used throughout this project.
 */

PapyrusWindow *getMainWindow();

QString outputTypeToString(OutputType outputType);
#endif // HELPERS_H
