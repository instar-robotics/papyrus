#ifndef SCRIPT_H
#define SCRIPT_H

#include "diagramscene.h"

#include <QString>
#include <QFile>
#include <QUuid>
#include <QTimer>

// Forward declaration because of recursive includes
class DiagramScene;

// Defines whether the time value is a frequency in Hz or a period in ms
enum TimeUnit {
    HZ,
    MS
};

Q_DECLARE_METATYPE(TimeUnit) // This allows convertion from/to QVariant

/**
 * @brief The Script class represents a neural scripts. An XML file is associated to it,
 * as well as the QGraphicsScene that contains its functions boxes
 */

class Script : public QObject
{
    Q_OBJECT
public:
    Script(DiagramScene *scene, const QString &name = "");

    void save();
    void autoSave();

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

public slots:
    void warnAboutModifiedScript();

private:
    DiagramScene *m_scene; // The associated scene for this script
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

signals:
    void displayStatusMessage(const QString &text);
};

#endif // SCRIPT_H
