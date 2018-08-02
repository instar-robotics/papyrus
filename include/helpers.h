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

/**
 * This file provides a set of helpers that are used throughout this project.
 */

PapyrusWindow *getMainWindow();

QString outputTypeToString(OutputType outputType);
OutputType stringToOutputType(QString str);

QString inputTypeToString(InputType inputType);
InputType stringToInputType(QString str);

QString timeUnitToString(TimeUnit unit);

bool canLink(OutputType from, InputType to);

void informUserAndCrash(const QString &text, const QString &title = QObject::tr("Papyrus is about to crash!"));

void rescaleSvgItem(QGraphicsSvgItem *svg, const QSizeF size, const QPointF pos, bool center = true);

void updateSizeIcon(DiagramBox *box);

bool areLinked(OutputSlot *oSlot, InputSlot *iSlot);

bool fileExists(const std::string& filename);

QList<QString> getKheopsNodes();

#endif // HELPERS_H