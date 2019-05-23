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

#include "propertiespanel.h"
#include "helpers.h"
#include "constants.h"
#include "connectivitywindow.h"
#include "constantdiagrambox.h"
#include "updateboxcommand.h"
#include "updatelinkcommand.h"
#include "updatezonecommand.h"
#include "updatescriptcommand.h"

#include <QVBoxLayout>
#include <QDebug>
#include <QLabel>
#include <QMessageBox>
#include <QToolButton>
#include <QScreen>

PropertiesPanel::PropertiesPanel(QWidget *parent) : QGroupBox(parent),
                                                    m_panelLayout(NULL),
                                                    m_scriptFrame(NULL),
                                                    m_scriptName(NULL),
                                                    m_timeLabel(NULL),
                                                    m_timeValue(NULL),
                                                    m_encrypt(NULL),
                                                    m_boxLayout(NULL),
                                                    m_boxFrame(NULL),
                                                    m_boxName(NULL),
                                                    m_boxOutputType(NULL),
                                                    m_boxMatrixShape(NULL),
                                                    m_rowsInput(NULL),
                                                    m_colsInput(NULL),
                                                    m_saveActivity(NULL),
                                                    m_publish(NULL),
                                                    m_topic(NULL),
                                                    m_linkLayout(NULL),
                                                    m_linkFrame(NULL),
                                                    m_linkType(NULL),
                                                    m_linkSecondary(NULL),
                                                    m_linkWeight(NULL),
                                                    m_linkValue(NULL),
                                                    m_linkConnectivity(nullptr),
                                                    m_linkRegexes(nullptr),
                                                    m_okBtn(NULL),
                                                    m_cancelBtn(NULL)
{
	setTitle(tr("Properties"));
	QVBoxLayout *m_panelLayout = new QVBoxLayout;
	m_scriptFrame = new QFrame;
	m_boxFrame = new QFrame;
	m_linkFrame = new QFrame;
	m_zoneFrame = new QFrame;
	m_okBtn = new QPushButton("OK");
	m_okBtn->setIcon(QIcon(":/icons/icons/check.svg"));
	m_okBtn->setToolTip(tr("Apply changes to the selected item"));
	m_cancelBtn = new QPushButton("Cancel");
	m_cancelBtn->setIcon(QIcon(":/icons/icons/cancel.svg"));
	m_cancelBtn->setIconSize(QSize(22, 22)); // Shrink the cancel icon size a little
	m_cancelBtn->setToolTip(tr("Discard changes and reset to the selected item's"));
	m_panelLayout->addWidget(m_scriptFrame);
	m_panelLayout->addWidget(m_boxFrame);
	m_panelLayout->addWidget(m_linkFrame);
	m_panelLayout->addWidget(m_zoneFrame);
	QHBoxLayout *btnsLayout = new QHBoxLayout;
	btnsLayout->addWidget(m_okBtn);
	btnsLayout->addWidget(m_cancelBtn);
	m_panelLayout->addLayout(btnsLayout);
	setLayout(m_panelLayout);

	// Create layout for script's frame
	QFormLayout *scriptLayout = new QFormLayout;
	QLabel scriptTitle(tr("Script Properties"));
	QFont scriptFont = scriptTitle.font();
	scriptFont.setBold(true);
	scriptTitle.setFont(scriptFont);
	m_scriptName = new QLabel;
	m_timeLabel = new QLabel(tr("Freq:"));
	m_timeValue = new PropDoubleSpinBox;
	m_timeUnit = new QComboBox;
	m_encrypt = new QCheckBox;

	// Parameterize the fields
	scriptLayout->setContentsMargins(0, 0, 0, 0);
	m_timeValue->setRange(MIN_TIME_VALUE, MAX_TIME_VALUE);
	m_timeValue->setDecimals(3);
	m_timeValue->setSuffix(" Hz");
	m_timeValue->setFixedWidth(130);
	m_timeUnit->addItem("Hz", HZ);
	m_timeUnit->addItem("ms", MS);
	connect(m_timeUnit, SIGNAL(currentIndexChanged(int)), SLOT(convertTimeValues(int)));
	m_timeUnit->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

	// Add fields to layout
	scriptLayout->addRow(&scriptTitle);
	scriptLayout->addRow(tr("Name:"), m_scriptName);
	scriptLayout->addRow(m_timeLabel, m_timeValue);
	scriptLayout->addRow(tr("Unit:"), m_timeUnit);
	scriptLayout->addRow(tr("Crypted:"), m_encrypt);

	m_scriptFrame->setLayout(scriptLayout);

	// Create the layout for the frames
	m_boxLayout = new QFormLayout;
	m_boxName = new QLabel;
	m_boxTitle = new PropLineEdit;
	m_boxOutputType = new QLabel;
	m_boxMatrixShape = new QLabel;
	m_rowsInput = new QSpinBox;
	m_colsInput = new QSpinBox;
	m_saveActivity = new QCheckBox(tr("Save Activity"));
	m_publish = new QCheckBox(tr("Publish output"));
	m_topic = new PropLineEdit;
	m_displayVisu = new QPushButton(tr("Display Visualization"));

	// Parameterize the fields
	m_boxLayout->setContentsMargins(0, 0, 0, 0); // Reduce inner margins due to lack of space
	m_boxName->setAlignment(Qt::AlignCenter);
	QFont f(m_boxName->font());
	f.setBold(true);
	m_boxName->setFont(f);
	m_boxTitle->setSizeHint(QSize(170, 35));
	m_topic->setSizeHint(QSize(170, 35));
	m_rowsInput->setRange(1, MAX_ROWS);
	m_colsInput->setRange(1, MAX_COLS);
	m_rowsInput->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_colsInput->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	connect(m_publish, SIGNAL(toggled(bool)), this, SLOT(toggleTopic(bool)));
	connect(m_topic, SIGNAL(textChanged(QString)), this, SLOT(onTopicChanged(QString)));

	// Add the fields to the layout
	m_boxLayout->addRow(m_boxName);
	m_boxLayout->addRow(tr("Title:"), m_boxTitle);
	m_boxLayout->addRow(tr("Type:"), m_boxOutputType);
	m_boxLayout->addRow(tr("Shape:"), m_boxMatrixShape);
	m_boxLayout->addRow(tr("Rows:"), m_rowsInput);
	m_boxLayout->addRow(tr("Cols:"), m_colsInput);
	m_boxLayout->addRow(m_saveActivity);
	m_boxLayout->addRow(m_publish);
	m_boxLayout->addRow(tr("Topic:"), m_topic);
	m_boxLayout->addRow(m_displayVisu);

	m_boxFrame->setLayout(m_boxLayout);

	// Create layout for the Link
	m_linkLayout = new QFormLayout;
	m_linkLayout->setContentsMargins(0, 0, 0, 0);
	m_linkType = new QLabel;
	m_linkType->setAlignment(Qt::AlignCenter);
	f = (m_linkType->font());
	f.setBold(true);
	m_linkType->setFont(f);

	m_linkSecondary = new QCheckBox(tr("Secondary"));
	m_linkWeight = new QDoubleSpinBox();
	m_linkWeight->setRange(MIN_WEIGHT, MAX_WEIGHT);
	m_linkWeight->setDecimals(LINKS_NB_DECIMALS);
	m_linkWeight->setFixedWidth(150);
	m_linkWeight->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_linkValue = new QLineEdit;
	m_linkConnectivity = new QComboBox;
	m_linkConnectivity->addItem("One", ONE_TO_ONE);
	m_linkConnectivity->addItem("All", ONE_TO_ALL);
	m_linkConnectivity->addItem("Neighbors", ONE_TO_NEI);
	connect(m_linkConnectivity, SIGNAL(currentIndexChanged(int)), this, SLOT(onConnectivityChanged(int)));
	m_linkRegexes = new QTextEdit;
	m_linkRegexes->setPlaceholderText(tr("Connectivity regexes"));

	m_linkLayout->addRow(m_linkType);
	m_linkLayout->addRow(tr("Weight:"), m_linkWeight);
	m_linkLayout->addRow(tr("Value:"), m_linkValue);
	m_linkLayout->addRow(m_linkSecondary);
	m_linkLayout->addRow(tr("One to"), m_linkConnectivity);
	m_linkLayout->addRow(m_linkRegexes);

	m_linkFrame->setLayout(m_linkLayout);

	m_linkLayout->setSizeConstraint(QLayout::SetFixedSize);

	// Create the layout for the zone
	m_zoneLayout = new QFormLayout;
	m_zoneLayout->setContentsMargins(0, 0, 0, 0);
	m_zoneTitle = new PropLineEdit;
	m_zoneColor = new SetColorButton;

	m_zoneTitle->setSizeHint(QSize(170, 35));
	m_zoneLayout->addRow(tr("Title:"), m_zoneTitle);
	m_zoneLayout->addRow(tr("Color:"), m_zoneColor);

	m_zoneFrame->setLayout(m_zoneLayout);

	// Decrease slightly the font size for the whole panel, because it's quite big
	QFont panelFont = font();
	panelFont.setPointSize(panelFont.pointSize() - 2);
	setFont(panelFont);

	// By default, hide the frames and the buttons
	hideAllFrames(true);
}

/**
 * @brief PropertiesPanel::hideAllFrames hides all frame from the properties panel, making it empty
 */
void PropertiesPanel::hideAllFrames(bool buttonsToo)
{
	m_linkFrame->hide();
	m_boxFrame->hide();
	m_scriptFrame->hide();
	m_zoneFrame->hide();

	if (buttonsToo) {
		m_okBtn->hide();
		m_cancelBtn->hide();
	}
}

QFrame *PropertiesPanel::boxFrame() const
{
	return m_boxFrame;
}

void PropertiesPanel::setBoxFrame(QFrame *boxFrame)
{
	m_boxFrame = boxFrame;
}

QFrame *PropertiesPanel::linkFrame() const
{
	return m_linkFrame;
}

void PropertiesPanel::setLinkFrame(QFrame *linkFrame)
{
	m_linkFrame = linkFrame;
}

QLabel *PropertiesPanel::boxName() const
{
	return m_boxName;
}

void PropertiesPanel::setBoxName(QLabel *boxName)
{
	m_boxName = boxName;
}

QSpinBox *PropertiesPanel::rowsInput() const
{
	return m_rowsInput;
}

QSpinBox *PropertiesPanel::colsInput() const
{
	return m_colsInput;
}

QPushButton *PropertiesPanel::okBtn() const
{
	return m_okBtn;
}

void PropertiesPanel::setOkBtn(QPushButton *okBtn)
{
	m_okBtn = okBtn;
}

QPushButton *PropertiesPanel::cancelBtn() const
{
	return m_cancelBtn;
}

void PropertiesPanel::setCancelBtn(QPushButton *cancelBtn)
{
	m_cancelBtn = cancelBtn;
}

QCheckBox *PropertiesPanel::saveActivity() const
{
	return m_saveActivity;
}

void PropertiesPanel::setSaveActivity(QCheckBox *saveActivity)
{
	m_saveActivity = saveActivity;
}

QDoubleSpinBox *PropertiesPanel::linkWeight() const
{
	return m_linkWeight;
}

QLabel *PropertiesPanel::scriptName() const
{
	return m_scriptName;
}

void PropertiesPanel::setScriptName(QLabel *scriptName)
{
	m_scriptName = scriptName;
}

PropDoubleSpinBox *PropertiesPanel::timeValue() const
{
	return m_timeValue;
}

void PropertiesPanel::setTimeValue(PropDoubleSpinBox *timeValue)
{
	m_timeValue = timeValue;
}

QComboBox *PropertiesPanel::timeUnit() const
{
	return m_timeUnit;
}

void PropertiesPanel::setTimeUnit(QComboBox *timeUnit)
{
	m_timeUnit = timeUnit;
}

QLineEdit *PropertiesPanel::linkValue() const
{
	return m_linkValue;
}

void PropertiesPanel::setLinkValue(QLineEdit *linkValue)
{
	m_linkValue = linkValue;
}

QPushButton *PropertiesPanel::displayVisu() const
{
	return m_displayVisu;
}

void PropertiesPanel::setDisplayVisu(QPushButton *displayVisu)
{
	m_displayVisu = displayVisu;
}

PropLineEdit *PropertiesPanel::zoneTitle() const
{
	return m_zoneTitle;
}

void PropertiesPanel::setZoneTitle(PropLineEdit *zoneTitle)
{
	m_zoneTitle = zoneTitle;
}

SetColorButton *PropertiesPanel::zoneColor() const
{
	return m_zoneColor;
}

void PropertiesPanel::setZoneColor(SetColorButton *zoneColor)
{
	m_zoneColor = zoneColor;
}

PropLineEdit *PropertiesPanel::boxTitle() const
{
	return m_boxTitle;
}

void PropertiesPanel::setBoxTitle(PropLineEdit *boxTitle)
{
	m_boxTitle = boxTitle;
}

QLabel *PropertiesPanel::boxMatrixShape() const
{
	return m_boxMatrixShape;
}

void PropertiesPanel::setBoxMatrixShape(QLabel *boxMatrixShape)
{
	m_boxMatrixShape = boxMatrixShape;
}

QCheckBox *PropertiesPanel::publish() const
{
	return m_publish;
}

void PropertiesPanel::setPublish(QCheckBox *publish)
{
	m_publish = publish;
}

PropLineEdit *PropertiesPanel::topic() const
{
	return m_topic;
}

void PropertiesPanel::setTopic(PropLineEdit *topic)
{
	m_topic = topic;
}

QCheckBox *PropertiesPanel::linkSecondary() const
{
	return m_linkSecondary;
}

void PropertiesPanel::setLinkSecondary(QCheckBox *linkSecondary)
{
	m_linkSecondary = linkSecondary;
}

QCheckBox *PropertiesPanel::encrypt() const
{
	return m_encrypt;
}

void PropertiesPanel::setEncrypt(QCheckBox *encrypt)
{
	m_encrypt = encrypt;
}

QComboBox *PropertiesPanel::linkConnectivity() const
{
	return m_linkConnectivity;
}

void PropertiesPanel::setLinkConnectivity(QComboBox *linkConnectivity)
{
	m_linkConnectivity = linkConnectivity;
}

QTextEdit *PropertiesPanel::linkRegexes() const
{
	return m_linkRegexes;
}

void PropertiesPanel::setLinkRegexes(QTextEdit *linkRegexes)
{
	m_linkRegexes = linkRegexes;
}

/**
 * @brief PropertiesPanel::displayBoxProperties updates the contents of the PropertiesPanel to
 * display the properties of the selected box
 */
void PropertiesPanel::displayBoxProperties(DiagramBox *box)
{
	if (box == NULL)
		informUserAndCrash(tr("Cannot display box's properties because box is null!"));

	bool isConstantBox = (dynamic_cast<ConstantDiagramBox *>(box) != nullptr);

	// Hide the other frames
	hideAllFrames();

	// Update the fields with the selected box
	m_boxName->setText(box->name());
	m_boxTitle->setText(box->title());
	m_boxTitle->setPlaceholderText(box->name());

	if (!isConstantBox) {
		// Check the "save activity" box according to the box's flag
		m_saveActivity->setChecked(box->saveActivity());

		// Check the "publish output" box according to the box's flag
		m_publish->setChecked(box->publish());

		// Get the topic name and enable / disable it based on the publish flag
		m_topic->setText(box->topic());
		m_topic->setEnabled(box->publish());
	}


	OutputType oType = box->outputType();

	if (oType == MATRIX) {
		m_boxOutputType->setText(tr("matrix"));
		m_rowsInput->setValue(box->rows());
		m_colsInput->setValue(box->cols());

		// Re-insert input for row and columns since it's a matrix
		QWidget *dimLabel = NULL;
		if ((dimLabel = m_boxLayout->labelForField(m_colsInput)))
			dimLabel->show();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'columns'"));
		m_rowsInput->show();

		if ((dimLabel = m_boxLayout->labelForField(m_rowsInput)))
			dimLabel->show();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'rows'"));
		m_colsInput->show();

		// Gray out appropriate fields based on the shape
		MatrixShape matrixShape = box->matrixShape();
		if (matrixShape == POINT) {
			m_colsInput->setEnabled(false);
			m_rowsInput->setEnabled(false);
		} else if (matrixShape == ROW_VECT) {
			m_rowsInput->setEnabled(false);
			m_colsInput->setEnabled(true);
		} else if (matrixShape == COL_VECT) {
			m_rowsInput->setEnabled(true);
			m_colsInput->setEnabled(false);
		} else {
			m_rowsInput->setEnabled(true);
			m_colsInput->setEnabled(true);
		}

		if (!isConstantBox) {
		// Show the shape of the box (since it's matrix)
		m_boxMatrixShape->setText(matrixShapeToString(box->matrixShape()));
		QWidget *shapeLabel = NULL;
		if ((shapeLabel = m_boxLayout->labelForField(m_boxMatrixShape)))
			shapeLabel->show();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'shape'"));
		m_boxMatrixShape->show();
		}
	} else if (oType == SCALAR) {
		m_boxOutputType->setText(tr("scalar"));

		// Hide input for rows and columns since it's a scalar
		QWidget *dimLabel = NULL;
		if ((dimLabel = m_boxLayout->labelForField(m_colsInput)))
			dimLabel->hide();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'columns'"));
		m_colsInput->hide();

		if ((dimLabel = m_boxLayout->labelForField(m_rowsInput)))
			m_rowsInput->hide();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'rows'"));
		dimLabel->hide();

		// Hide the shape field
		QWidget *shapeLabel = NULL;
		if ((shapeLabel = m_boxLayout->labelForField(m_boxMatrixShape)))
			shapeLabel->hide();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'shape'"));
		m_boxMatrixShape->hide();
	} else if (oType == STRING) {
		m_boxOutputType->setText(tr("string"));

		// Hide input for rows and columns since it's not a matrix
		QWidget *dimLabel = NULL;
		if ((dimLabel = m_boxLayout->labelForField(m_colsInput)))
			dimLabel->hide();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'columns'"));
		m_colsInput->hide();

		if ((dimLabel = m_boxLayout->labelForField(m_rowsInput)))
			m_rowsInput->hide();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'rows'"));
		dimLabel->hide();

		// Hide the shape field
		QWidget *shapeLabel = NULL;
		if ((shapeLabel = m_boxLayout->labelForField(m_boxMatrixShape)))
			shapeLabel->hide();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'shape'"));
		m_boxMatrixShape->hide();
	} else {
		informUserAndCrash(tr("Unsupported output type for a box. Supported types are MATRIX and SCALAR"));
	}

	// Hide fields for constant box
	QWidget *topicLabel = m_boxLayout->labelForField(m_topic);
	if (topicLabel == nullptr)
		informUserAndCrash(tr("Failed to fetch label for topic name."));

	if (isConstantBox) {
		m_saveActivity->hide();
		m_publish->hide();
		topicLabel->hide();
		m_topic->hide();
		m_displayVisu->hide();
	} else {
		m_saveActivity->show();
		m_publish->show();
		topicLabel->show();
		m_topic->show();
		m_displayVisu->show();
	}

	// Show the box frame and the buttons
	m_boxFrame->show();
	m_okBtn->show();
	m_cancelBtn->show();
}

/**
 * @brief PropertiesPanel::displayLinkProperties updates the contents of the PropertiesPanel to
 * display the properties of the selected box
 */
void PropertiesPanel::displayLinkProperties(Link *link)
{
	if (link == NULL)
		informUserAndCrash(tr("Cannot display link's properties because link is null!"));

	// Hide the other frames
	hideAllFrames();

	InputType linkType = link->to()->inputType();

	// Populate the panels with the link's properties
	m_linkType->setText(inputTypeToString(linkType));
	m_linkSecondary->setChecked(link->secondary());

	QWidget *weightFieldLabel = m_linkLayout->labelForField(m_linkWeight);
	QWidget *valueFieldLabel = m_linkLayout->labelForField(m_linkValue);

	if (weightFieldLabel == NULL)
		informUserAndCrash(tr("Failed to fetch label for field 'weight'"));

	if (valueFieldLabel == NULL)
		informUserAndCrash(tr("Failed to fetch label for field 'value'"));

	// If the link is not a string field, display the weight input
	if (!link->isStringLink()) {
		m_linkValue->hide();
		valueFieldLabel->hide();
		m_linkWeight->setValue(link->weight());
		weightFieldLabel->show();
		m_linkWeight->show();
	}
	// If the link is a string link, then display its value instead
	else {
		m_linkWeight->hide();
		weightFieldLabel->hide();
		m_linkValue->setText(link->value());
		valueFieldLabel->show();
		m_linkValue->show();
	}

	// Disable secondary if the link is self-looping (necessary a secondary link)
	if (link->selfLoop())
		m_linkSecondary->setEnabled(false);
	else
		m_linkSecondary->setEnabled(true);

	// Show the connectivity button for links of type MATRIX_MATRIX
	QWidget *linkConnectivityLabel = m_linkLayout->labelForField(m_linkConnectivity);
	if (linkType == MATRIX_MATRIX) {
		m_linkConnectivity->show();

		if (linkConnectivityLabel != nullptr)
			linkConnectivityLabel->show();

		// Hide the regexes (will be shown only when necessary)
		m_linkRegexes->hide();
		m_linkRegexes->clear();

		switch (link->connectivity()) {
			case ONE_TO_ONE:
				m_linkConnectivity->setCurrentIndex(0);
			break;

			case ONE_TO_ALL:
				m_linkConnectivity->setCurrentIndex(1);
			break;

			case ONE_TO_NEI:
				m_linkConnectivity->setCurrentIndex(2);
				m_linkRegexes->show();
				m_linkRegexes->setPlainText(link->regexes());
			break;

			default:
				emit displayStatusMessage(tr("Unsupported connectivity."));
				m_linkConnectivity->hide();
				if (linkConnectivityLabel != nullptr)
					linkConnectivityLabel->hide();
		}
	} else {
		m_linkConnectivity->hide();

		if (linkConnectivityLabel != nullptr)
			linkConnectivityLabel->hide();

		m_linkRegexes->hide();
	}

	// Show the link frame
	m_linkFrame->show();
	m_okBtn->show();
	m_cancelBtn->show();
}

/**
 * @brief PropertiesPanel::displayScriptProperties shows the script's properties in the properties
 * panel (information such as script name, script frequency, etc.)
 * @param script
 */
void PropertiesPanel::displayScriptProperties(Script *script)
{
	if (script == NULL)
		informUserAndCrash(tr("Cannot display script's properties because script is null!"));

	// Hide the other frames
	hideAllFrames();

	// Populate fields
	m_scriptName->setText(script->name());
	m_timeValue->setValue(script->timeValue());
	switch (script->timeUnit()) {
		case HZ:
			m_timeUnit->setCurrentIndex(0);
		break;

		case MS:
			m_timeUnit->setCurrentIndex(1);
		break;

		default:
			qWarning() << "Unsupported time Unit";
	}
	m_encrypt->setChecked(script->encrypt());

	// Show the script's frame
	m_scriptFrame->show();
	m_okBtn->show();
	m_cancelBtn->show();

	m_timeValue->adjustSize();
}

void PropertiesPanel::displayZoneProperties(Zone *zone)
{
	if (zone == nullptr)
		informUserAndCrash(tr("Cannot display comment zone's properties because script is null!"));

	// Hide all other frames
	hideAllFrames();

	// Populate fields
	m_zoneTitle->setText(zone->title());
	m_zoneColor->setColor(zone->color());

	// Show the zone frame
	m_zoneFrame->show();
	m_okBtn->show();
	m_cancelBtn->show();
}

void PropertiesPanel::convertTimeValues(int idx)
{
	m_timeValue->setValue(1000.0 / m_timeValue->value());

	TimeUnit unit = m_timeUnit->itemData(idx).value<TimeUnit>();

	if (unit == MS) {
		m_timeLabel->setText(tr("Period:"));
		m_timeValue->setSuffix(" ms");
	} else if (unit == HZ) {
		m_timeLabel->setText(tr("Freq:"));
		m_timeValue->setSuffix(" Hz");
	} else {
		informUserAndCrash(tr("Unsupported time unit. Supported units are Hz and ms."));
	}
}

void PropertiesPanel::toggleTopic(bool isChecked)
{
	m_topic->setEnabled(isChecked);
}

/**
 * @brief PropertiesPanel::onTopicChanged fires when the user edits the value in a box's topic name
 * field. It is used to check if what the user is entering is valid or not, and color-code it.
 * @param topic the new value entered by the user for the box' topic
 */
void PropertiesPanel::onTopicChanged(const QString &topic)
{
	if (isTopicNameValid(topic)) {
		m_topic->setStyleSheet("color: black;");
	} else {
		m_topic->setStyleSheet("color: red;");
	}
}

/**
 * @brief PropertiesPanel::onConnectivityChanged is called when the user changes the connectivity
 * type of a MATRIX_MATRIX @Link. It is used to show or hide the QTextEdit in which connectivity
 * regexes are typed in.
 * @param idx
 */
void PropertiesPanel::onConnectivityChanged(int idx)
{
	Q_UNUSED(idx);

	Connectivity connectivity = m_linkConnectivity->currentData().value<Connectivity>();

	// If the user selected ONE_TO_NEI, then show the text edit and populate with current value
	if (connectivity == ONE_TO_NEI) {
		m_linkRegexes->clear(); // Since we don't have access to the Link, start with blank
		m_linkRegexes->show();
	}
	// If the user selected another connectivity, hide the text edit
	else {
		m_linkRegexes->hide();
	}
}

/**
 * @brief PropertiesPanel::updateBoxProperties is called when the user clicked "OK" after changing
 * some properties of the selected box. It updates the selected box's properties based on the
 * contents of the fields in the properties panel
 * @param box
 */
void PropertiesPanel::updateBoxProperties(DiagramBox *box)
{
	UpdateBoxCommand *updateCommand = new UpdateBoxCommand(this, box);

	DiagramScene *dScene = dynamic_cast<DiagramScene *>(box->scene());
	if (dScene == nullptr) {
		qWarning() << "Cannot update box's properties: no scene!";
		return;
	}

	if (dScene->undoStack() == nullptr) {
		qWarning() << "Cannot update box's properties: no undo stack!";
		return;
	}

	dScene->undoStack()->push(updateCommand);
}

void PropertiesPanel::updateLinkProperties(Link *link)
{
	UpdateLinkCommand *updateCommand = new UpdateLinkCommand(this, link);

	DiagramScene *dScene = dynamic_cast<DiagramScene *>(link->to()->box()->scene());
	if (dScene == nullptr) {
		qWarning() << "Cannot update link's properties: no scene!";
		return;
	}

	if (dScene->undoStack() == nullptr) {
		qWarning() << "Cannot update link's properties: no undo stack!";
		return;
	}

	dScene->undoStack()->push(updateCommand);
}

/**
 * @brief PropertiesPanel::updateScriptProperties updates the script's properties (such as RT Token)
 * when "OK" is pressed in the properties panel
 * @param script
 */
void PropertiesPanel::updateScriptProperties(Script *script)
{
	if (script == NULL)
		informUserAndCrash(tr("Cannot update script's properties: script is null!"));

	UpdateScriptCommand *updateCommand = new UpdateScriptCommand(this, script);
	if (script->scene() == nullptr) {
		qWarning() << "Cannot update script's properties: no scene!";
		return;
	}

	script->scene()->undoStack()->push(updateCommand);

	/*
	script->setTimeValue(m_timeValue->value());
	script->setTimeUnit(m_timeUnit->currentData().value<TimeUnit>());
	script->setEncrypt(m_encrypt->isChecked());
	*/
}

void PropertiesPanel::updateZoneProperties(Zone *zone)
{
	if (zone == nullptr)
		informUserAndCrash(tr("Cannot update zone's properties: script is null!"));

	UpdateZoneCommand *updateCommand = new UpdateZoneCommand(this, zone);

	DiagramScene *dScene = dynamic_cast<DiagramScene *>(zone->scene());
	if (dScene == nullptr) {
		qWarning() << "Cannot update link's properties: no scene!";
		return;
	}

	if (dScene->undoStack() == nullptr) {
		qWarning() << "Cannot update link's properties: no undo stack!";
		return;
	}

	dScene->undoStack()->push(updateCommand);
	/*
	if (zone == nullptr)
		informUserAndCrash(tr("Cannot update zone's properties: script is null!"));

	zone->setTitle(m_zoneTitle->text());
	zone->setColor(m_zoneColor->color());
	*/
}

/**
 * @brief PropertiesPanel::keyPressEvent catches key presses in order to binds ENTER and ESCAPE
 * to OK and CANCEL actions
 * @param event
 */
void PropertiesPanel::keyPressEvent(QKeyEvent *event)
{
	int key = event->key();

	if (key == Qt::Key_Escape) {
		emit escapePressed();
	} else if (key == Qt::Key_Enter || key == Qt::Key_Return) {
		// Note: "Enter" is the keypad's enter key and "Return" is the main keyboard's enter key
		emit enterPressed();
	}

	QGroupBox::keyPressEvent(event);
}
