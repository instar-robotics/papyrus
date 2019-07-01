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

#ifndef SCRIPT_H
#define SCRIPT_H

#include "diagramscene.h"
#include "types.h"
#include "rossession.h"

#include <QString>
#include <QFile>
#include <QDir>
#include <QUuid>
#include <QTimer>

// Forward declaration because of recursive includes
class DiagramScene;

Q_DECLARE_METATYPE(TimeUnit) // This allows convertion from/to QVariant

/**
 * @brief The Script class represents a neural scripts. An XML file is associated to it,
 * as well as the @QGraphicsScene that contains its functions boxes
 */

class Script : public QObject
{
	Q_OBJECT

public:
	Script(DiagramScene *scene, const QString &name = "");
	~Script();

	void save(const QString &basePath = QDir::homePath(),
	          bool isAutoSave = false);

	void updateTextStyle();
	void runOrPause();
	void run();
	void pause();
	void stop();
	ScriptStatus queryScriptStatus();
	void setupROSSession();

	QString name() const;
	void setName(const QString &name);

	QString filePath() const;
	void setFilePath(const QString &filePath);

	DiagramScene *scene() const;

	bool modified() const;

	void setStatusModified(bool isModified);

	bool isInvalid() const;
	void setIsInvalid(bool isInvalid);

	double timeValue() const;
	void setTimeValue(double timeValue);

	TimeUnit timeUnit() const;
	void setTimeUnit(const TimeUnit &timeUnit);

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	bool encrypt() const;
	void setEncrypt(bool encrypt);

	ROSSession *rosSession() const;
	void setRosSession(ROSSession *rosSession);

	bool isActiveScript() const;
	void setIsActiveScript(bool isActiveScript);

	bool hasTab() const;
	void setHasTab(bool hasTab);

	bool isRunning() const;
	void setIsRunning(bool isRunning);

	bool isPaused() const;
	void setIsPaused(bool isPaused);

	QString nodeName() const;
	void setNodeName(const QString &nodeName);

	void setTabIdx(int tabIdx);

	int tabIdx() const;

public slots:
	void warnAboutModifiedScript();

private:
	DiagramScene *m_scene; // The associated scene for this script
	bool m_hasTab; // Tells whether the scripts has a tab in the tabwidget or not
	int m_tabIdx;  // The index of this script's tab in the main tab widget
	ROSSession *m_rosSession; // The associated ROS Session for this script
	QString m_name;        // Pretty name of the script (to display in tabs for instance)
	QString m_nodeName;    // The ROS node name of the script
	QString m_filePath;    // Path of the (XML) file in which to save this script
	bool m_modified;       // Whether there was some changes since last save
	bool m_isInvalid;      // Whether this script is currently invalid (and thus prevent saving)
	double m_timeValue;    // The RT Token time (either frequency or period)
	TimeUnit m_timeUnit;   // Whether the time value is a frequency or a period
	QUuid m_uuid;          // UUID for the RT Token (needed by kheops)
	QTimer m_modifiedNotifTimer; // Timer to display a system tray notification when unsaved for more than X minutes
	bool m_encrypt;        // Whether the XML script should be encrypted on save (to protect IP)
	std::string m_key;     // AES Key used to encrypt the file
	std::string m_iv;      // AES IV used to encrypt the file
	bool m_isActiveScript; // Tells this script if it's the currently active one
	bool m_isRunning;      // Tells whether this script is running (launched)
	bool m_isPaused;       // Tells whether this script is paused while running

private slots:
	void onROSSessionMessage(const QString &msg, MessageUrgency urgency = MSG_INFO);
	void onScriptResumed();
	void onScriptPaused();
	void onScriptStopped();
	void onTimeElapsed(int h, int m, int s, int ms);
	void handleRTTokenMessage(ScopeMessage *rtTokenMessage);

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void scriptResumed();
	void scriptPaused();
	void scriptStopped();
	void timeElapsed(int h, int m, int s, int ms);
	void rtTokenWarning(bool warning, int idx);
};

#endif // SCRIPT_H
