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

#ifndef PROPERTIESPANEL_H
#define PROPERTIESPANEL_H

#include "diagrambox.h"
#include "link.h"
#include "script.h"
#include "zone.h"
#include "setcolorbutton.h"
#include "proplineedit.h"
#include "propdoublespinbox.h"
#include "diagramscene.h"

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
#include <QTextEdit>
#include <QRadioButton>

/**
 * @brief The PropertiesPanel class is used to display (and modify) the properties of selected
 * objects (such as slots, boxes, links, etc.)
 */

class PropertiesPanel : public QGroupBox
{
	Q_OBJECT

public:
	explicit PropertiesPanel(QWidget *parent = nullptr);

	void hideAllFrames(bool buttonsToo = false);
	void hideBoxFrame();
	void hideLinkFrame();
	void updateBoxProperties(DiagramBox *box);
	void updateLinkProperties(Link *link);
	void updateScriptProperties(Script *script);
	void updateZoneProperties(Zone *zone);

	void hideOkBtn();
	void hideCancelBtn();

	void keyPressEvent(QKeyEvent *event);

	QString getBoxTitle();
	int getBoxRows();
	int getBoxCols();
	bool getBoxSaveActivity();
	bool getBoxPublish();
	QString getBoxTopic();
	bool getBoxUseValue();
	QString getBoxRowsVariable();
	QString getBoxColsVariable();

	bool getLinkSecondary();
	qreal getLinkWeight();
	QString getLinkValue();
	Connectivity getLinkConnectivity();
	QString getLinkRegexes();
	bool getLinkUseValue();

	QString getZoneTitle();
	QColor getZoneColor();

	void setTopicNameColor(QColor color);

	qreal getScriptTimeValue();
	void setScriptTimeValue(qreal timeValue);

	TimeUnit getScriptTimeUnit();
	void setScriptTimeUnit(TimeUnit timeUnit);

	bool getScriptEncrypt();

	QPushButton *displayVisu();

	QPushButton *okBtn();

	QPushButton *cancelBtn();

	void hideLayoutItem(QWidget *tohide, bool alsoHideLabel = true, QFormLayout *fromLayout = nullptr);
	void showLayoutItem(QWidget *toShow, bool alsoShowLabel = true, QFormLayout *fromLayout = nullptr);

private:
	DiagramScene *m_activeScene; // The currently visible scene

	// Script
	QVBoxLayout *m_panelLayout;  // The properties panel's main layout
	QFrame m_scriptFrame;       // Container for script's properties
	QLabel m_scriptName;        // Label used to change the script (and tab) name
	QLabel m_timeLabel;         // Contains either "frequency" or "period"
	PropDoubleSpinBox m_timeValue; // Used to input the script's frequency (or period)
	QComboBox m_timeUnit;       // Used to select the unit (in Hz or ms)
	QCheckBox m_encrypt;        // Whether or not the file is encrypted on save
	QPushButton m_weightsSaveBtn; // Button used to save script's weight to default location
	QPushButton m_weightsLoadBtn; // Button used to load script's weigth from default location

	// Box
	QFormLayout *m_boxLayout;        // Layout for the box properties (access needed to hide rows)
	QFrame m_boxFrame;               // Container for box's properties
	QLabel m_boxName;                // Display the name of the box
	PropLineEdit m_boxTitle;         // Allow to see or change the box's custom name
	QLabel m_boxOutputType;          // Display the box's output type (scalar, matrix)
	QLabel m_boxMatrixShape;         // Display the shape of the function (when matrix)
	QSpinBox m_rowsInput;            // Spin box to input number of rows in the output (if matrix)
	QSpinBox m_colsInput;            // Spin box to input number of columns in the output (if matrix)
	QLineEdit m_rowsVarInput;        // Variable input field for rows
	QLineEdit m_colsVarInput;        // Variable input field for cols
	QCheckBox m_saveActivity;        // To enable saving the activity of the box
	QCheckBox m_publish;             // To enable publish the output of the function
	PropLineEdit m_topic;            // To input the topic name for publishing
	QPushButton m_displayVisu;       // Display the box's data vizualisation
	QRadioButton m_radioBoxValue;    // Radio button to select value for box's size
	QRadioButton m_radioBoxVariable; // Radio button to select variable for box's size

	// Link
	QFormLayout *m_linkLayout;        // Layout for the link properties (access needed to hide rows)
	QFrame m_linkFrame;               // Container for link's properties
	QLabel m_linkType;                // Display the type of the link
	QCheckBox m_linkSecondary;        // Will display if the link is secondary or not
	QDoubleSpinBox m_linkWeight;      // Spin box to set the weight of the link
	QLineEdit m_linkValue;            // Text field to enter the link's value (for string links or variable input)
	QComboBox m_linkConnectivity;     // Change the connectivity of the link (for MATRIX_MATRIX)
	QTextEdit m_linkRegexes;          // The ONE_TO_NEI regexes to define neighbors
	QRadioButton m_radioLinkValue;    // Radio button to select value for input
	QRadioButton m_radioLinkVariable; // Radio button to select variable for input

	// Zone
	QFormLayout *m_zoneLayout;     // Contains the layout to display comment zone's properties
	QFrame m_zoneFrame;           // Container for zone's properties
	PropLineEdit m_zoneTitle;        // The comment zone's title
	SetColorButton m_zoneColor;     // Holds the color of the comment zone

	// Will contain the sizes of connected matrix when the link is SPARSE_MATRIX
	QSize m_inputSize;
	QSize m_outputSize;

	// Buttons
	QPushButton m_okBtn;      // Button used to validate changes in parameters
	QPushButton m_cancelBtn;  // Button used to discard changes in parameters and restore current

public slots:
	void displayBoxProperties(DiagramBox *box);
	void displayLinkProperties(Link *link);
	void displayScriptProperties(Script *script);
	void displayZoneProperties(Zone *zone);
	void convertTimeValues(int idx);
	void toggleTopic(bool isChecked);

private slots:
	void onTopicChanged(const QString &topic);
	void onConnectivityChanged(int idx);
	void onWeightsSaveClicked(bool);
	void onWeightsLoadClicked(bool);
	void onBoxRadioValueToggled(bool isSelected);
	void onLinkRadioValueToggled(bool isSelected);
signals:
	void enterPressed();
	void escapePressed();
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
};

#endif // PROPERTIESPANEL_H
