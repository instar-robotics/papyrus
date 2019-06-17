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

#ifndef HELPERS_H
#define HELPERS_H

#include "types.h"
#include "papyruswindow.h"
#include "slot.h"

#include <ros/ros.h>

#include <QApplication>
#include <QString>
#include <QGraphicsSvgItem>
#include <QList>
#include <QDebug>
#include <QColor>

/**
 * This file provides a set of helpers that are used throughout this project.
 */

PapyrusWindow *getMainWindow();

QString outputTypeToString(const OutputType &outputType);
OutputType stringToOutputType(const QString &str);

QString inputTypeToString(const InputType &inputType);
InputType stringToInputType(const QString &str);

QString timeUnitToString(const TimeUnit &unit);

bool canLink(const OutputType &from, const InputType &to);

bool shapesMatch(DiagramBox *from, InputSlot *to, LinkInvalidReason *reason = nullptr);

void informUserAndCrash(const QString &text, const QString &title = QObject::tr("Papyrus is about to crash!"));

void rescaleSvgItem(QGraphicsSvgItem *svg, const QSizeF &size, const QPointF &pos, bool center = true);

bool areLinked(OutputSlot *oSlot, InputSlot *iSlot);

bool isFull(InputSlot *iSlot);

bool fileExists(const std::string& filename);

QList<QString> getKheopsNodes();

QString snakeCaseToPretty(const QString &str);

QString connectivityToString(const Connectivity &conn);

Connectivity stringToConnectivity(const QString &str);

QString mkTopicName(const QString &scriptName, const QString &topicName);

QString sanitizeTopicName(const QString &name);

bool isTopicNameValid(const QString &name);

QString ensureSlashPrefix(const QString &name);

QColor getTypeColor(const InputType inputType);

QColor getTypeColor(const OutputType outputType);

QString mkFilenameFromScript(const QString &scriptName);

qreal qrealAbsMax(const qreal a, qreal b);

MatrixShape stringToMatrixShape(const QString &str);

QString matrixShapeToString(const MatrixShape shape);

VisuType stringToVisuType(const QString &str);

QString visuTypeToString(const VisuType &visuType);

#endif // HELPERS_H
