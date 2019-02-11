#ifndef PROPERTIESPANEL_H
#define PROPERTIESPANEL_H

#include "diagrambox.h"
#include "link.h"
#include "script.h"
#include "zone.h"
#include "setcolorbutton.h"
#include "proplineedit.h"
#include "propdoublespinbox.h"

#include <QGroupBox>
#include <QLineEdit>
#include <QFrame>
#include <QLabel>
#include <QSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QFormLayout>

/**
 * @brief The PropertiesPanel class is used to display (and modify) the properties of selected
 * objects (such as slots, boxes, links, etc.)
 */

class PropertiesPanel : public QGroupBox
{
	Q_OBJECT

public:
	explicit PropertiesPanel(QWidget *parent = 0);

	void hideAllFrames(bool buttonsToo = false);
	void updateBoxProperties(DiagramBox *box);
	void updateLinkProperties(Link *link);
	void updateScriptProperties(Script *script);
	void updateZoneProperties(Zone *zone);

	void keyPressEvent(QKeyEvent *event);

	QFrame *boxFrame() const;
	void setBoxFrame(QFrame *boxFrame);

	QFrame *linkFrame() const;
	void setLinkFrame(QFrame *linkFrame);

	QLabel *boxName() const;
	void setBoxName(QLabel *boxName);

	QSpinBox *rowsInput() const;

	QSpinBox *colsInput() const;

	QPushButton *okBtn() const;
	void setOkBtn(QPushButton *okBtn);

	QPushButton *cancelBtn() const;
	void setCancelBtn(QPushButton *cancelBtn);

	QCheckBox *saveActivity() const;
	void setSaveActivity(QCheckBox *saveActivity);

	QDoubleSpinBox *linkWeight() const;

	QLabel *scriptName() const;
	void setScriptName(QLabel *scriptName);

	PropDoubleSpinBox *timeValue() const;
	void setTimeValue(PropDoubleSpinBox *timeValue);

	QComboBox *timeUnit() const;
	void setTimeUnit(QComboBox *timeUnit);

	QLineEdit *linkValue() const;
	void setLinkValue(QLineEdit *linkValue);

	QPushButton *displayVisu() const;
	void setDisplayVisu(QPushButton *displayVisu);

	PropLineEdit *zoneTitle() const;
	void setZoneTitle(PropLineEdit *zoneTitle);

	SetColorButton *zoneColor() const;
	void setZoneColor(SetColorButton *zoneColor);

	PropLineEdit *boxTitle() const;
	void setBoxTitle(PropLineEdit *boxTitle);

	QLabel *boxMatrixShape() const;
	void setBoxMatrixShape(QLabel *boxMatrixShape);

	QCheckBox *publish() const;
	void setPublish(QCheckBox *publish);

	PropLineEdit *topic() const;
	void setTopic(PropLineEdit *topic);

	QCheckBox *linkSecondary() const;
	void setLinkSecondary(QCheckBox *linkSecondary);

	QCheckBox *encrypt() const;
	void setEncrypt(QCheckBox *encrypt);

private:
	QVBoxLayout *m_panelLayout;  // The properties panel's main layout
	QFrame *m_scriptFrame;       // Container for script's properties
	QLabel *m_scriptName;        // Label used to change the script (and tab) name
	QLabel *m_timeLabel;         // Contains either "frequency" or "period"
	PropDoubleSpinBox *m_timeValue; // Used to input the script's frequency (or period)
	QComboBox *m_timeUnit;       // Used to select the unit (in Hz or ms)
	QCheckBox *m_encrypt;        // Whether or not the file is encrypted on save

	QFormLayout *m_boxLayout;  // Layout for the box properties (access needed to hide rows)
	QFrame *m_boxFrame;        // Container for box's properties
	QLabel *m_boxName;         // Display the name of the box
	PropLineEdit *m_boxTitle;     // Allow to see or change the box's custom name
	QLabel *m_boxOutputType;   // Display the box's output type (scalar, matrix)
	QLabel *m_boxMatrixShape;  // Display the shape of the function (when matrix)
	QSpinBox *m_rowsInput;     // Spin box to input number of rows in the output (if matrix)
	QSpinBox *m_colsInput;     // Spin box to input number of columns in the output (if matrix)
	QCheckBox *m_saveActivity; // To enable saving the activity of the box
	QCheckBox *m_publish;      // To enable publish the output of the function
	PropLineEdit *m_topic;        // To input the topic name for publishing
	QPushButton *m_displayVisu; // (TEMP) display the box's data vizualisation

	QFormLayout *m_linkLayout;    // Layout for the link properties (access needed to hide rows)
	QFrame *m_linkFrame;          // Container for link's properties
	QLabel *m_linkType;           // Display the type of the link
	QCheckBox *m_linkSecondary;   // Will display if the link is secondary or not
	QDoubleSpinBox *m_linkWeight; // Spin box to set the weight of the link
	QLineEdit *m_linkValue;       // Text field to enter the link's value (for string links)

	QFormLayout *m_zoneLayout;     // Contains the layout to display comment zone's properties
	QFrame *m_zoneFrame;           // Container for zone's properties
	PropLineEdit *m_zoneTitle;        // The comment zone's title
	SetColorButton *m_zoneColor;     // Holds the color of the comment zone

	// Will contain the sizes of connected matrix when the link is SPARSE_MATRIX
	QSize m_inputSize;
	QSize m_outputSize;

	QPushButton *m_okBtn;      // Button used to validate changes in parameters
	QPushButton *m_cancelBtn;  // Button used to discard changes in parameters and restore current

public slots:
	void displayBoxProperties(DiagramBox *box);
	void displayLinkProperties(Link *link);
	void displayScriptProperties(Script *script);
	void displayZoneProperties(Zone *zone);
	void convertTimeValues(int idx);
	void toggleTopic(bool isChecked);

private slots:
	void onTopicChanged(const QString &topic);

signals:
	void enterPressed();
	void escapePressed();
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
};

#endif // PROPERTIESPANEL_H
