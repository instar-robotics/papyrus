#include "messagehandler.h"

#include <QByteArray>

void coloredMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    QByteArray localMsg = msg.toLocal8Bit();

    switch (type) {
    case QtDebugMsg:
        fprintf(stderr, "\033[1;35m[DEBUG]:\033[0;35m %s (%s:%u, %s)\033[0m\n", localMsg.constData(), context.file, context.line, context.function);
        break;
    case QtInfoMsg:
        fprintf(stderr, "\033[1;37m[INFO]:\033[0;37m %s (%s:%u, %s)\033[0m\n", localMsg.constData(), context.file, context.line, context.function);
        break;
    case QtWarningMsg:
        fprintf(stderr, "\033[1;33m[WARNING]:\033[0;33m %s (%s:%u, %s)\033[0m\n", localMsg.constData(), context.file, context.line, context.function);
        break;
    case QtCriticalMsg:
        fprintf(stderr, "\033[1;31m[CRITICAL]:\033[0;31m %s (%s:%u, %s)\033[0m\n", localMsg.constData(), context.file, context.line, context.function);
        break;
    case QtFatalMsg:
        fprintf(stderr, "\033[1;41m[FATAL]: %s (%s:%u, %s)\033[0m\n", localMsg.constData(), context.file, context.line, context.function);
        break;
    }
}
