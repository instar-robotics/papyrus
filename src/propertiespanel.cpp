﻿/*
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
                                                    m_activeScene(nullptr),
                                                    m_panelLayout(nullptr),
                                                    m_scriptFrame(this),
                                                    m_scriptName(this),
                                                    m_timeLabel(tr("Freq:"), this),
                                                    m_timeValue(this),
                                                    m_timeUnit(this),
                                                    m_encrypt(this),
                                                    m_weightsSaveBtn(this),
                                                    m_weightsLoadBtn(this),
                                                    m_boxLayout(nullptr),
                                                    m_boxFrame(this),
                                                    m_boxName(this),
                                                    m_boxOutputType(this),
                                                    m_boxMatrixShape(this),
                                                    m_rowsInput(this),
                                                    m_colsInput(this),
                                                    m_saveActivity(tr("Save Activity"), this),
                                                    m_publish(tr("Publish output"), this),
                                                    m_topic(this),
                                                    m_displayVisu(tr("Visualize Activity"), this),
                                                    m_choseVisuType(this),
                                                    m_linkLayout(nullptr),
                                                    m_linkFrame(this),
                                                    m_linkType(this),
                                                    m_linkSecondary(tr("Secondary"), this),
                                                    m_linkWeight(this),
                                                    m_linkValue(this),
                                                    m_linkConnectivity(this),
                                                    m_linkRegexes(this),
                                                    m_zoneFrame(this),
                                                    m_zoneTitle(this),
                                                    m_zoneColor(this),
                                                    m_okBtn(tr("OK"), this),
                                                    m_cancelBtn(tr("Cancel"), this)
{
	setTitle(tr("Properties"));
	QVBoxLayout *m_panelLayout = new QVBoxLayout;
	m_okBtn.setIcon(QIcon(":/icons/icons/check.svg"));
	m_okBtn.setToolTip(tr("Apply changes to the selected item"));
	m_cancelBtn.setIcon(QIcon(":/icons/icons/cancel.svg"));
	m_cancelBtn.setIconSize(QSize(22, 22)); // Shrink the cancel icon size a little
	m_cancelBtn.setToolTip(tr("Discard changes and reset to the selected item's"));
	m_panelLayout->addWidget(&m_scriptFrame);
	m_panelLayout->addWidget(&m_boxFrame);
	m_panelLayout->addWidget(&m_linkFrame);
	m_panelLayout->addWidget(&m_zoneFrame);
	QHBoxLayout *btnsLayout = new QHBoxLayout;
	btnsLayout->addWidget(&m_okBtn);
	btnsLayout->addWidget(&m_cancelBtn);
	m_panelLayout->addLayout(btnsLayout);
	setLayout(m_panelLayout);

	// Create layout for script's frame
	QFormLayout *scriptLayout = new QFormLayout;
	QLabel scriptTitle(tr("Script Properties"));
	QFont scriptFont = scriptTitle.font();
	scriptFont.setBold(true);
	scriptTitle.setFont(scriptFont);

	// Parameterize the fields
	scriptLayout->setContentsMargins(0, 0, 0, 0);
	m_timeValue.setRange(MIN_TIME_VALUE, MAX_TIME_VALUE);
	m_timeValue.setDecimals(3);
	m_timeValue.setSuffix(" Hz");
	m_timeValue.setFixedWidth(130);
	m_timeUnit.addItem("Hz", HZ);
	m_timeUnit.addItem("ms", MS);
	connect(&m_timeUnit, SIGNAL(currentIndexChanged(int)), SLOT(convertTimeValues(int)));
	m_timeUnit.setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_weightsSaveBtn.setText("Weights");
	m_weightsSaveBtn.setIcon(QIcon(":/icons/icons/save-weights.svg"));
	m_weightsSaveBtn.setToolTip(tr("<strong>Save</strong> weights in default location."));
	m_weightsLoadBtn.setText("Weights");
	m_weightsLoadBtn.setIcon(QIcon(":/icons/icons/load-weights.svg"));
	m_weightsLoadBtn.setToolTip(tr("<strong>Load</strong> weights from default location."));

	// Add fields to layout
	scriptLayout->addRow(&scriptTitle);
	scriptLayout->addRow(tr("Name:"), &m_scriptName);
	scriptLayout->addRow(&m_timeLabel, &m_timeValue);
	scriptLayout->addRow(tr("Unit:"), &m_timeUnit);
	scriptLayout->addRow(&m_weightsSaveBtn, &m_weightsLoadBtn);
	scriptLayout->addRow(tr("Crypted:"), &m_encrypt);

	m_scriptFrame.setLayout(scriptLayout);

	connect(&m_weightsSaveBtn, SIGNAL(clicked(bool)), this, SLOT(onWeightsSaveClicked(bool)));
	connect(&m_weightsLoadBtn, SIGNAL(clicked(bool)), this, SLOT(onWeightsLoadClicked(bool)));

	// Parameterize the fields
	m_boxLayout = new QGridLayout;
	m_boxLayout->setContentsMargins(0, 0, 0, 0); // Reduce inner margins due to lack of space
	m_boxName.setAlignment(Qt::AlignCenter);
	QFont f(m_boxName.font());
	f.setBold(true);
	m_boxName.setFont(f);
	m_boxTitle.setSizeHint(QSize(130, 35));
	m_topic.setSizeHint(QSize(130, 35));
	m_rowsInput.setRange(1, MAX_ROWS);
	m_colsInput.setRange(1, MAX_COLS);
	m_rowsInput.setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_colsInput.setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_changeParameters.setText(tr("Change parameters"));
//	m_displayVisu.setText(tr("Visualize Activity"));
	connect(&m_publish, SIGNAL(toggled(bool)), this, SLOT(toggleTopic(bool)));
	connect(&m_topic, SIGNAL(textChanged(QString)), this, SLOT(onTopicChanged(QString)));
	connect(&m_displayVisu, SIGNAL(clicked(bool)), this, SLOT(onDisplayVisuClicked(bool)));
	connect(&m_changeParameters, SIGNAL(clicked(bool)), this, SLOT(onChangeParametersClicked(bool)));

	// Fill the labels
	m_boxTitleLabel.setText(tr("Title:"));
	m_boxOutputTypeLabel.setText(tr("Type:"));
	m_boxMatrixShapeLabel.setText(tr("Shape:"));
	m_rowsInputLabel.setText(tr("Rows:"));
	m_colsInputLabel.setText(tr("Cols:"));
	m_topicLabel.setText(tr("Topic:"));

	// Add the fields to the layout
	int i = 0;
	m_boxLayout->addWidget(&m_boxName,i,0,1,2);
	m_boxLayout->addWidget(&m_boxTitleLabel,++i,0,1,1);
	m_boxLayout->addWidget(&m_boxTitle,i,1,1,1);
	m_boxLayout->addWidget(&m_boxOutputTypeLabel,++i,0,1,1);
	m_boxLayout->addWidget(&m_boxOutputType,i,1,1,1);
	m_boxLayout->addWidget(&m_boxMatrixShapeLabel,++i,0,1,1);
	m_boxLayout->addWidget(&m_boxMatrixShape,i,1,1,1);
	m_boxLayout->addWidget(&m_rowsInputLabel,++i,0,1,1);
	m_boxLayout->addWidget(&m_rowsInput,i,1,1,1);
	m_boxLayout->addWidget(&m_colsInputLabel,++i,0,1,1);
	m_boxLayout->addWidget(&m_colsInput,i,1,1,1);
	m_boxLayout->addWidget(&m_saveActivity,++i,0,1,1);
	m_boxLayout->addWidget(&m_publish,i,1,1,1);
	m_boxLayout->addWidget(&m_topicLabel,++i,0,1,2);
	m_boxLayout->addWidget(&m_topic,++i,0,1,2);
	m_boxLayout->addWidget(&m_displayVisu,++i,0,1,2);
	m_boxLayout->addWidget(&m_choseVisuType,++i,0,1,1);
	m_boxLayout->addWidget(&m_changeParameters,i,1,1,1);

	m_boxFrame.setLayout(m_boxLayout);
	addVisuChoices();
	connect(this, SIGNAL(changeCurrentVisuType(QString)), &m_choseVisuType, SLOT(setCurrentText(QString)));

	// Create layout for the Link
	m_linkLayout = new QFormLayout;
	m_linkLayout->setContentsMargins(0, 0, 0, 0);
	m_linkType.setAlignment(Qt::AlignCenter);
	f = (m_linkType.font());
	f.setBold(true);
	m_linkType.setFont(f);

	m_linkWeight.setRange(MIN_WEIGHT, MAX_WEIGHT);
	m_linkWeight.setDecimals(LINKS_NB_DECIMALS);
	m_linkWeight.setFixedWidth(150);
	m_linkWeight.setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_linkConnectivity.addItem("One", ONE_TO_ONE);
	m_linkConnectivity.addItem("All", ONE_TO_ALL);
	m_linkConnectivity.addItem("Neighbors", ONE_TO_NEI);
	connect(&m_linkConnectivity, SIGNAL(currentIndexChanged(int)), this, SLOT(onConnectivityChanged(int)));
	m_linkRegexes.setPlaceholderText(tr("Connectivity regexes"));

	m_linkLayout->addRow(&m_linkType);
	m_linkLayout->addRow(tr("Weight:"), &m_linkWeight);
	m_linkLayout->addRow(tr("Value:"), &m_linkValue);
	m_linkLayout->addRow(&m_linkSecondary);
	m_linkLayout->addRow(tr("One to"), &m_linkConnectivity);
	m_linkLayout->addRow(&m_linkRegexes);

	m_linkFrame.setLayout(m_linkLayout);

	m_linkLayout->setSizeConstraint(QLayout::SetFixedSize);

	// Create the layout for the zone
	m_zoneLayout = new QFormLayout;
	m_zoneLayout->setContentsMargins(0, 0, 0, 0);

	m_zoneTitle.setSizeHint(QSize(170, 35));
	m_zoneLayout->addRow(tr("Title:"), &m_zoneTitle);
	m_zoneLayout->addRow(tr("Color:"), &m_zoneColor);

	m_zoneFrame.setLayout(m_zoneLayout);

	// Decrease slightly the font size for the whole panel, because it's quite big
	QFont panelFont = font();
	panelFont.setPointSize(panelFont.pointSize() - 2);
	setFont(panelFont);

	// By default, hide the frames and the buttons
	hideAllFrames(true);
}
void PropertiesPanel::onDisplayVisuClicked(bool)
{
	emit displayVisu(stringToVisuType(m_choseVisuType.currentText()));
}

/**
 * @brief PropertiesPanel::hideAllFrames hides all frame from the properties panel, making it empty
 */
void PropertiesPanel::hideAllFrames(bool buttonsToo)
{
	m_linkFrame.hide();
	m_boxFrame.hide();
	m_scriptFrame.hide();
	m_zoneFrame.hide();

	if (buttonsToo) {
		m_okBtn.hide();
		m_cancelBtn.hide();
	}
}

void PropertiesPanel::hideBoxFrame()
{
	m_boxFrame.hide();
}

void PropertiesPanel::hideLinkFrame()
{
	m_linkFrame.hide();
}

QPushButton *PropertiesPanel::cancelBtn()
{
	return &m_cancelBtn;
}

void PropertiesPanel::addVisuChoices()
{
	int i = 0;
	m_choseVisuType.insertItem(i++,visuTypeToString(THERMAL_2D));
	m_choseVisuType.insertSeparator(i++);

	m_choseVisuType.insertItem(i++,visuTypeToString(SURFACE_3D));
	m_choseVisuType.insertItem(i++,visuTypeToString(BAR_CHART_3D));
	m_choseVisuType.insertItem(i++,visuTypeToString(CONE_CHART_3D));
	m_choseVisuType.insertItem(i++,visuTypeToString(WIREFRAME_3D));
	m_choseVisuType.insertSeparator(i++);

	m_choseVisuType.insertItem(i++,visuTypeToString(CROWN_3D));
	m_choseVisuType.insertItem(i++,visuTypeToString(BAR_CIRCLE_3D));
	m_choseVisuType.insertSeparator(i++);

	m_choseVisuType.insertItem(i++,visuTypeToString(SURFACE_POLAR_3D));
	m_choseVisuType.insertItem(i++,visuTypeToString(BAR_POLAR_3D));
	m_choseVisuType.insertItem(i++,visuTypeToString(WIREFRAME_POLAR_3D));
	m_choseVisuType.insertSeparator(i++);

	m_choseVisuType.insertItem(i++,visuTypeToString(COMPASS_3D));
}

VisuType PropertiesPanel::visuType()
{
	return stringToVisuType(m_choseVisuType.currentText());
}

void PropertiesPanel::updateVisuTypeChoices(int rows, int cols)
{
	QModelIndex index;
	QVariant var;
	for(int i = 0; i < m_choseVisuType.count(); i++)
	{
		VisuType type = stringToVisuType(m_choseVisuType.itemText(i));
		if(type != UNKNOWN_VISU_TYPE)
		{
			index = m_choseVisuType.model()->index(i,0);
			if(doesVisuFit(type, rows, cols))
				var = QVariant(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
			else
				var = QVariant(Qt::NoItemFlags);
			m_choseVisuType.model()->setData(index, var, Qt::UserRole-1);
		}
	}
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
	m_boxName.setText(box->name());
	m_boxTitle.setText(box->title());
	m_boxTitle.setPlaceholderText(box->name());
	emit changeCurrentVisuType(visuTypeToString(box->visuType()));

	if (!isConstantBox) {
		// Check the "save activity" box according to the box's flag
		m_saveActivity.setChecked(box->saveActivity());

		// Check the "publish output" box according to the box's flag
		m_publish.setChecked(box->publish());

		// Get the topic name and enable / disable it based on the publish flag
		m_topic.setText(box->topic());
		m_topic.setEnabled(box->publish());
	}


	OutputType oType = box->outputType();

	if (oType == MATRIX) {
		m_boxOutputType.setText(tr("matrix"));
		m_rowsInput.setValue(box->rows());
		m_colsInput.setValue(box->cols());

		// Re-insert input for row and columns since it's a matrix
		QWidget *dimLabel = NULL;
		if ((dimLabel = &m_colsInputLabel))
			dimLabel->show();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'columns'"));
		m_rowsInput.show();

		if ((dimLabel = &m_rowsInputLabel))
			dimLabel->show();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'rows'"));
		m_colsInput.show();

		// Gray out appropriate fields based on the shape
		MatrixShape matrixShape = box->matrixShape();
		if (matrixShape == POINT) {
			m_colsInput.setEnabled(false);
			m_rowsInput.setEnabled(false);
		} else if (matrixShape == ROW_VECT) {
			m_rowsInput.setEnabled(false);
			m_colsInput.setEnabled(true);
		} else if (matrixShape == COL_VECT) {
			m_rowsInput.setEnabled(true);
			m_colsInput.setEnabled(false);
		} else {
			m_rowsInput.setEnabled(true);
			m_colsInput.setEnabled(true);
		}

		if (!isConstantBox) {
		// Show the shape of the box (since it's matrix)
		m_boxMatrixShape.setText(matrixShapeToString(box->matrixShape()));
		QWidget *shapeLabel = NULL;
		if ((shapeLabel = &m_boxMatrixShapeLabel))
			shapeLabel->show();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'shape'"));
		m_boxMatrixShape.show();
		}
	} else if (oType == SCALAR) {
		m_boxOutputType.setText(tr("scalar"));

		// Hide input for rows and columns since it's a scalar
		QWidget *dimLabel = NULL;
		if ((dimLabel = &m_colsInputLabel))
			dimLabel->hide();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'columns'"));
		m_colsInput.hide();

		if ((dimLabel = &m_rowsInputLabel))
			m_rowsInput.hide();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'rows'"));
		dimLabel->hide();

		// Hide the shape field
		QWidget *shapeLabel = NULL;
		if ((shapeLabel = &m_boxMatrixShapeLabel))
			shapeLabel->hide();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'shape'"));
		m_boxMatrixShape.hide();
	} else if (oType == STRING) {
		m_boxOutputType.setText(tr("string"));

		// Hide input for rows and columns since it's not a matrix
		QWidget *dimLabel = NULL;
		if ((dimLabel = &m_colsInputLabel))
			dimLabel->hide();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'columns'"));
		m_colsInput.hide();

		if ((dimLabel = &m_rowsInputLabel))
			m_rowsInput.hide();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'rows'"));
		dimLabel->hide();

		// Hide the shape field
		QWidget *shapeLabel = NULL;
		if ((shapeLabel = &m_boxMatrixShapeLabel))
			shapeLabel->hide();
		else
			informUserAndCrash(tr("Failed to fetch label for field 'shape'"));
		m_boxMatrixShape.hide();
	} else {
		informUserAndCrash(tr("Unsupported output type for a box. Supported types are MATRIX and SCALAR"));
	}

	// Hide fields for constant box
	QWidget *topicLabel = &m_topicLabel;
	if (topicLabel == nullptr)
		informUserAndCrash(tr("Failed to fetch label for topic name."));

	if (isConstantBox) {
		m_saveActivity.hide();
		m_publish.hide();
		topicLabel->hide();
		m_topic.hide();
		m_displayVisu.hide();
		m_choseVisuType.hide();
		m_changeParameters.hide();
	} else {
		m_saveActivity.show();
		m_publish.show();
		topicLabel->show();
		m_topic.show();
		m_displayVisu.show();
		m_choseVisuType.show();
		m_changeParameters.show();
	}

	// Show the box frame and the buttons
	m_boxFrame.show();
	m_okBtn.show();
	m_cancelBtn.show();
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
	m_linkType.setText(inputTypeToString(linkType));
	m_linkSecondary.setChecked(link->secondary());

	QWidget *weightFieldLabel = m_linkLayout->labelForField(&m_linkWeight);
	QWidget *valueFieldLabel = m_linkLayout->labelForField(&m_linkValue);

	if (weightFieldLabel == NULL)
		informUserAndCrash(tr("Failed to fetch label for field 'weight'"));

	if (valueFieldLabel == NULL)
		informUserAndCrash(tr("Failed to fetch label for field 'value'"));

	// If the link is not a string field, display the weight input
	if (!link->isStringLink()) {
		m_linkValue.hide();
		valueFieldLabel->hide();
		m_linkWeight.setValue(link->weight());
		weightFieldLabel->show();
		m_linkWeight.show();
	}
	// If the link is a string link, then display its value instead
	else {
		m_linkWeight.hide();
		weightFieldLabel->hide();
		m_linkValue.setText(link->value());
		valueFieldLabel->show();
		m_linkValue.show();
	}

	// Disable secondary if the link is self-looping (necessary a secondary link)
	if (link->selfLoop())
		m_linkSecondary.setEnabled(false);
	else
		m_linkSecondary.setEnabled(true);

	// Show the connectivity button for links of type MATRIX_MATRIX
	QWidget *linkConnectivityLabel = m_linkLayout->labelForField(&m_linkConnectivity);
	if (linkType == MATRIX_MATRIX) {
		m_linkConnectivity.show();

		if (linkConnectivityLabel != nullptr)
			linkConnectivityLabel->show();

		// Hide the regexes (will be shown only when necessary)
		m_linkRegexes.hide();
		m_linkRegexes.clear();

		switch (link->connectivity()) {
			case ONE_TO_ONE:
				m_linkConnectivity.setCurrentIndex(0);
			break;

			case ONE_TO_ALL:
				m_linkConnectivity.setCurrentIndex(1);
			break;

			case ONE_TO_NEI:
				m_linkConnectivity.setCurrentIndex(2);
				m_linkRegexes.show();
				m_linkRegexes.setPlainText(link->regexes());
			break;

			default:
				emit displayStatusMessage(tr("Unsupported connectivity."));
				m_linkConnectivity.hide();
				if (linkConnectivityLabel != nullptr)
					linkConnectivityLabel->hide();
		}
	} else {
		m_linkConnectivity.hide();

		if (linkConnectivityLabel != nullptr)
			linkConnectivityLabel->hide();

		m_linkRegexes.hide();
	}

	// Show the link frame
	m_linkFrame.show();
	m_okBtn.show();
	m_cancelBtn.show();
}

/**
 * @brief PropertiesPanel::displayScriptProperties shows the script's properties in the properties
 * panel (information such as script name, script frequency, etc.)
 * @param script
 */
void PropertiesPanel::displayScriptProperties(Script *script)
{
	if (script == nullptr)
		informUserAndCrash(tr("Cannot display script's properties because script is null!"));

	// Set the current active scene
	m_activeScene = script->scene();

	// Hide the other frames
	hideAllFrames();

	// Populate fields
	m_scriptName.setText(script->name());
	m_timeValue.setValue(script->timeValue());
	switch (script->timeUnit()) {
		case HZ:
			m_timeUnit.setCurrentIndex(0);
		break;

		case MS:
			m_timeUnit.setCurrentIndex(1);
		break;

		default:
			qWarning() << "Unsupported time Unit";
	}
	m_encrypt.setChecked(script->encrypt());

	// Show the script's frame
	m_scriptFrame.show();
	m_okBtn.show();
	m_cancelBtn.show();

	m_timeValue.adjustSize();
}

void PropertiesPanel::displayZoneProperties(Zone *zone)
{
	if (zone == nullptr)
		informUserAndCrash(tr("Cannot display comment zone's properties because script is null!"));

	// Hide all other frames
	hideAllFrames();

	// Set the zone's scene to the zone color button
	m_zoneColor.setCurrentScene(dynamic_cast<DiagramScene *>(zone->scene()));

	// Populate fields
	m_zoneTitle.setText(zone->title());
	m_zoneColor.setColor(zone->color());

	// Show the zone frame
	m_zoneFrame.show();
	m_okBtn.show();
	m_cancelBtn.show();
}

void PropertiesPanel::convertTimeValues(int idx)
{
	m_timeValue.setValue(1000.0 / m_timeValue.value());

	TimeUnit unit = m_timeUnit.itemData(idx).value<TimeUnit>();

	if (unit == MS) {
		m_timeLabel.setText(tr("Period:"));
		m_timeValue.setSuffix(" ms");
	} else if (unit == HZ) {
		m_timeLabel.setText(tr("Freq:"));
		m_timeValue.setSuffix(" Hz");
	} else {
		informUserAndCrash(tr("Unsupported time unit. Supported units are Hz and ms."));
	}
}

void PropertiesPanel::toggleTopic(bool isChecked)
{
	m_topic.setEnabled(isChecked);
}

void PropertiesPanel::onChangeParametersClicked(bool)
{
	emit changeParameters(stringToVisuType(m_choseVisuType.currentText()));
}

/**
 * @brief PropertiesPanel::onTopicChanged fires when the user edits the value in a box's topic name
 * field. It is used to check if what the user is entering is valid or not, and color-code it.
 * @param topic the new value entered by the user for the box' topic
 */
void PropertiesPanel::onTopicChanged(const QString &topic)
{
	if (isTopicNameValid(topic)) {
		m_topic.setStyleSheet("color: black;");
	} else {
		m_topic.setStyleSheet("color: red;");
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

	Connectivity connectivity = m_linkConnectivity.currentData().value<Connectivity>();

	// If the user selected ONE_TO_NEI, then show the text edit and populate with current value
	if (connectivity == ONE_TO_NEI) {
		m_linkRegexes.clear(); // Since we don't have access to the Link, start with blank
		m_linkRegexes.show();
	}
	// If the user selected another connectivity, hide the text edit
	else {
		m_linkRegexes.hide();
	}
}

/**
 * @brief PropertiesPanel::onWeightsSaveClicked ask the script's to save its weights if its running
 */
void PropertiesPanel::onWeightsSaveClicked(bool)
{
	if (m_activeScene == nullptr) {
		emit displayStatusMessage(tr("Cannot save script's weights: no active scene!"), MSG_WARNING);
		return;
	}

	if (m_activeScene->script() == nullptr) {
		emit displayStatusMessage(tr("Could not save script's weights: active scene has no script!"), MSG_WARNING);
		return;
	}

	// Make sure the script is running
	if (!m_activeScene->script()->isRunning()) {
		emit displayStatusMessage(tr("Cannot save script's weights: script is not running!"), MSG_WARNING);
		return;
	}

	if (m_activeScene->script()->saveWeights())
		emit displayStatusMessage(tr("Weights saved successfully!"));
	else
		emit displayStatusMessage(tr("Failed to save weights."), MSG_ERROR);
}

/**
 * @brief PropertiesPanel::onWeightsSaveClicked ask the script's to load its weights if its running
 */
void PropertiesPanel::onWeightsLoadClicked(bool)
{
	if (m_activeScene == nullptr) {
		emit displayStatusMessage(tr("Cannot load script's weights: no active scene!"), MSG_WARNING);
		return;
	}

	if (m_activeScene->script() == nullptr) {
		emit displayStatusMessage(tr("Could not load script's weights: active scene has no script!"), MSG_WARNING);
		return;
	}

	// Make sure the script is running
	if (!m_activeScene->script()->isRunning()) {
		emit displayStatusMessage(tr("Cannot load script's weights: script is not running!"), MSG_WARNING);
		return;
	}

	if (m_activeScene->script()->loadWeights(""))
		emit displayStatusMessage(tr("Weights loaded successfully!"));
	else
		emit displayStatusMessage(tr("Failed to load weights."), MSG_ERROR);
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
	box->setVisuType(stringToVisuType(m_choseVisuType.currentText()));
	DiagramScene *dScene = dynamic_cast<DiagramScene *>(box->scene());
	if (dScene == nullptr) {
		qWarning() << "Cannot update box's properties: no scene!";
		return;
	}

//	if (dScene->undoStack() == nullptr) {
//		qWarning() << "Cannot update box's properties: no undo stack!";
//		return;
//	}
	dScene->undoStack().push(updateCommand);
}

void PropertiesPanel::updateLinkProperties(Link *link)
{
	UpdateLinkCommand *updateCommand = new UpdateLinkCommand(this, link);

	DiagramScene *dScene = dynamic_cast<DiagramScene *>(link->to()->box()->scene());
	if (dScene == nullptr) {
		qWarning() << "Cannot update link's properties: no scene!";
		return;
	}

//	if (dScene->undoStack() == nullptr) {
//		qWarning() << "Cannot update link's properties: no undo stack!";
//		return;
//	}

	dScene->undoStack().push(updateCommand);
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

	script->scene()->undoStack().push(updateCommand);

	/*
	script->setTimeValue(m_timeValue.value());
	script->setTimeUnit(m_timeUnit.currentData().value<TimeUnit>());
	script->setEncrypt(m_encrypt.isChecked());
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

//	if (dScene->undoStack() == nullptr) {
//		qWarning() << "Cannot update link's properties: no undo stack!";
//		return;
//	}

	dScene->undoStack().push(updateCommand);
	/*
	if (zone == nullptr)
		informUserAndCrash(tr("Cannot update zone's properties: script is null!"));

	zone->setTitle(m_zoneTitle.text());
	zone->setColor(m_zoneColor.color());
	*/
}

void PropertiesPanel::hideOkBtn()
{
	m_okBtn.hide();
}

void PropertiesPanel::hideCancelBtn()
{
	m_cancelBtn.hide();
}

QPushButton *PropertiesPanel::okBtn()
{
	return &m_okBtn;
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

QString PropertiesPanel::getBoxTitle()
{
	return m_boxTitle.text();
}

int PropertiesPanel::getBoxRows()
{
	return m_rowsInput.value();
}

int PropertiesPanel::getBoxCols()
{
	return m_colsInput.value();
}

bool PropertiesPanel::getBoxSaveActivity()
{
	return m_saveActivity.isChecked();
}

bool PropertiesPanel::getBoxPublish()
{
	return m_publish.isChecked();
}

QString PropertiesPanel::getBoxTopic()
{
	return m_topic.text();
}

bool PropertiesPanel::getLinkSecondary()
{
	return m_linkSecondary.isChecked();
}

qreal PropertiesPanel::getLinkWeight()
{
	return m_linkWeight.value();
}

QString PropertiesPanel::getLinkValue()
{
	return m_linkValue.text();
}

Connectivity PropertiesPanel::getLinkConnectivity()
{
	return m_linkConnectivity.currentData().value<Connectivity>();
}

QString PropertiesPanel::getLinkRegexes()
{
	return m_linkRegexes.toPlainText();
}

QString PropertiesPanel::getZoneTitle()
{
	return m_zoneTitle.text();
}

QColor PropertiesPanel::getZoneColor()
{
	return m_zoneColor.color();
}

void PropertiesPanel::setTopicNameColor(QColor color)
{
	m_topic.setStyleSheet(QString("color: rgb(%1, %2, %3);")
	                      .arg(color.red())
	                      .arg(color.green())
	                      .arg(color.blue()));
}

qreal PropertiesPanel::getScriptTimeValue()
{
	return m_timeValue.value();
}

void PropertiesPanel::setScriptTimeValue(qreal timeValue)
{
	m_timeValue.setValue(timeValue);
}

TimeUnit PropertiesPanel::getScriptTimeUnit()
{
	return m_timeUnit.currentData().value<TimeUnit>();
}

void PropertiesPanel::setScriptTimeUnit(TimeUnit timeUnit)
{
	m_timeUnit.setCurrentIndex(timeUnit);
}

bool PropertiesPanel::getScriptEncrypt()
{
	return m_encrypt.isChecked();
}

QPushButton *PropertiesPanel::displayVisu()
{
	return &m_displayVisu;
}
