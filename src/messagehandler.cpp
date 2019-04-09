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

#include "messagehandler.h"

#include <QByteArray>

void coloredMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
	Q_UNUSED(context);

	QByteArray localMsg = msg.toLocal8Bit();

	switch (type) {
		case QtDebugMsg:
//        fprintf(stderr, "\033[1;35m[DEBUG]:\033[0;35m %s (%s:%u, %s)\033[0m\n", localMsg.constData()), context.file, context.line, context.function);
			fprintf(stderr, "\033[1;35m[DEBUG]:\033[0;35m %s\033[0m\n", localMsg.constData());
		break;
		case QtInfoMsg:
//        fprintf(stderr, "\033[1;37m[INFO]:\033[0;37m %s (%s:%u, %s)\033[0m\n", localMsg.constData(), context.file, context.line, context.function);
			fprintf(stderr, "\033[1;37m[INFO]:\033[0;37m %s\033[0m\n", localMsg.constData());
		break;
		case QtWarningMsg:
//        fprintf(stderr, "\033[1;33m[WARNING]:\033[0;33m %s (%s:%u, %s)\033[0m\n", localMsg.constData(), context.file, context.line, context.function);
			fprintf(stderr, "\033[1;33m[WARNING]:\033[0;33m %s\033[0m\n", localMsg.constData());
		break;
		case QtCriticalMsg:
//        fprintf(stderr, "\033[1;31m[CRITICAL]:\033[0;31m %s (%s:%u, %s)\033[0m\n", localMsg.constData(), context.file, context.line, context.function);
			fprintf(stderr, "\033[1;31m[CRITICAL]:\033[0;31m %s\033[0m\n", localMsg.constData());
		break;
		case QtFatalMsg:
//        fprintf(stderr, "\033[1;41m[FATAL]: %s (%s:%u, %s)\033[0m\n", localMsg.constData(), context.file, context.line, context.function);
			fprintf(stderr, "\033[1;41m[FATAL]: %s\033[0m\n", localMsg.constData());
		break;
	}
}
