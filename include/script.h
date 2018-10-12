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

	void save(const QString &descriptionPath,
	          const QString &basePath = QDir::homePath(),
	          bool isAutoSave = false);

	void updateTextStyle();

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

public slots:
	void warnAboutModifiedScript();

private:
	DiagramScene *m_scene; // The associated scene for this script
	bool m_hasTab; // Tells whether the scripts has a tab in the tabwidget or not
	ROSSession *m_rosSession; // The associated ROS Session for this script
	QString m_name;        // Pretty name of the script (to display in tabs for instance)
	QString m_filePath;    // Path of the (XML) file in which to save this script
	bool m_modified;       // Whether there was some changes since last save
	bool m_isInvalid;      // Whether this script is currently invalid (and thus prevent saving)
	double m_timeValue;    // The RT Token time (either frequency or period)
	TimeUnit m_timeUnit;   // Whether the time value is a frequency or a period
	QUuid m_uuid;          // UUID for the RT Token (needed by kheops)
	QTimer *m_modifiedNotifTimer; // Timer to display a system tray notification when unsaved for more than X minutes
	bool m_encrypt;        // Whether the XML script should be encrypted on save (to protect IP)
	std::string m_key;     // AES Key used to encrypt the file
	std::string m_iv;      // AES IV used to encrypt the file
	bool m_isActiveScript; // Tells this script if it's the currently active one

private slots:
	void onROSSessionMessage(const QString &msg, MessageUrgency urgency = MSG_INFO);
	void onScriptResumed();
	void onScriptPaused();
	void onScriptStopped();
	void onTimeElapsed(int h, int m, int s, int ms);

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void scriptResumed();
	void scriptPaused();
	void scriptStopped();
	void timeElapsed(int h, int m, int s, int ms);
};

#endif // SCRIPT_H
