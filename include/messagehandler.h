#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <QApplication>
#include <QString>

void coloredMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg);

#endif //MESSAGE_HANDLER_H
