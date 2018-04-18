#include "propertiespanel.h"
#include "helpers.h"
#include "constants.h"

#include <QVBoxLayout>
#include <QDebug>
#include <QLabel>
#include <QMessageBox>

PropertiesPanel::PropertiesPanel(QWidget *parent) : QGroupBox(parent),
                                                    m_scriptFrame(NULL),
                                                    m_scriptName(NULL),
                                                    m_timeLabel(NULL),
                                                    m_timeValue(NULL),
                                                    m_boxLayout(NULL),
                                                    m_boxFrame(NULL),
                                                    m_boxName(NULL),
                                                    m_boxOutputType(NULL),
                                                    m_rowsInput(NULL),
                                                    m_colsInput(NULL),
                                                    m_saveActivity(NULL),
                                                    m_linkLayout(NULL),
                                                    m_linkFrame(NULL),
                                                    m_linkType(NULL),
                                                    m_linkOperation(NULL),
                                                    m_linkSecondary(NULL),
                                                    m_linkWeight(NULL),
                                                    m_okBtn(NULL),
                                                    m_cancelBtn(NULL)
{
    setTitle(tr("Properties"));
    QVBoxLayout *layout = new QVBoxLayout;
    m_scriptFrame = new QFrame;
    m_boxFrame = new QFrame;
    m_linkFrame = new QFrame;
    m_okBtn = new QPushButton("OK");
    m_okBtn->setIcon(QIcon(":/icons/icons/check.svg"));
    m_okBtn->setToolTip(tr("Apply changes to the selected item"));
    m_cancelBtn = new QPushButton("Cancel");
    m_cancelBtn->setIcon(QIcon(":/icons/icons/cancel.svg"));
    m_cancelBtn->setIconSize(QSize(22, 22)); // Shrink the cancel icon size a little
    m_cancelBtn->setToolTip(tr("Discard changes and reset to the selected item's"));
    layout->addWidget(m_scriptFrame);
    layout->addWidget(m_boxFrame);
    layout->addWidget(m_linkFrame);
    QHBoxLayout *btnsLayout = new QHBoxLayout;
    btnsLayout->addWidget(m_okBtn);
    btnsLayout->addWidget(m_cancelBtn);
    layout->addLayout(btnsLayout);
    setLayout(layout);

    // Create layout for script's frame
    QFormLayout *scriptLayout = new QFormLayout;
    QLabel scriptTitle(tr("Script Properties"));
    QFont scriptFont = scriptTitle.font();
    scriptFont.setBold(true);
    scriptTitle.setFont(scriptFont);
    m_scriptName = new QLabel;
    m_timeLabel = new QLabel(tr("Freq:"));
    m_timeValue = new QDoubleSpinBox;
    m_timeUnit = new QComboBox;

    // Parameterize the fields
    scriptLayout->setContentsMargins(0, 0, 0, 0);
    m_timeValue->setRange(MIN_TIME_VALUE, MAX_TIME_VALUE);
    m_timeValue->setDecimals(3);
    m_timeValue->setSuffix(" Hz");
    m_timeUnit->addItem("Hz", HZ);
    m_timeUnit->addItem("ms", MS);
    connect(m_timeUnit, SIGNAL(currentIndexChanged(int)), SLOT(convertTimeValues(int)));
    m_timeValue->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_timeUnit->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    // Add fields to layout
    scriptLayout->addRow(&scriptTitle);
    scriptLayout->addRow(tr("Name:"), m_scriptName);
    scriptLayout->addRow(m_timeLabel, m_timeValue);
    scriptLayout->addRow(tr("Unit:"), m_timeUnit);

    m_scriptFrame->setLayout(scriptLayout);

    // Create the layout for the frames
    m_boxLayout = new QFormLayout;
    m_boxName = new QLabel;
    m_boxOutputType = new QLabel;
    m_rowsInput = new QSpinBox;
    m_colsInput = new QSpinBox;
    m_saveActivity = new QCheckBox(tr("Save Activity"));

    // Parameterize the fields
    m_boxLayout->setContentsMargins(0, 0, 0, 0); // Reduce inner margins due to lack of space
    m_boxName->setAlignment(Qt::AlignCenter);
    QFont f(m_boxName->font());
    f.setBold(true);
    m_boxName->setFont(f);
    m_rowsInput->setRange(1, MAX_ROWS);
    m_colsInput->setRange(1, MAX_COLS);
    m_rowsInput->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_colsInput->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    // Add the fields to the layout
    m_boxLayout->addRow(m_boxName);
    m_boxLayout->addRow(tr("Type:"), m_boxOutputType);
    m_boxLayout->addRow(tr("Rows:"), m_rowsInput);
    m_boxLayout->addRow(tr("Cols:"), m_colsInput);
    m_boxLayout->addRow(m_saveActivity);

    m_boxFrame->setLayout(m_boxLayout);

    m_linkLayout = new QFormLayout;
    m_linkLayout->setContentsMargins(0, 0, 0, 0);
    m_linkType = new QLabel;
    m_linkType->setAlignment(Qt::AlignCenter);
    f = (m_linkType->font());
    f.setBold(true);
    m_linkType->setFont(f);
    m_linkOperation = new QComboBox();
    // WARNING: order must respect the order definition of the enum, because of the setCurrentIndex()
    m_linkOperation->addItem(tr("Input * Weight"), OP_PRODUCT);
    m_linkOperation->addItem(tr("Input + Weight"), OP_ADDITION);
    m_linkOperation->addItem(tr("Input - Weight"), OP_SUBTRACTION);
    m_linkOperation->addItem(tr("Input / Weight"), OP_DIVISION);
    m_linkOperation->setFixedWidth(150);

    m_linkSecondary = new QCheckBox(tr("Secondary"));
    m_linkSecondary->setEnabled(false); // For the moment, only display the information
    m_linkWeight = new QDoubleSpinBox();
    m_linkWeight->setRange(MIN_WEIGHT, MAX_WEIGHT);
    m_linkWeight->setDecimals(6);
    m_linkWeight->setFixedWidth(150);
    m_linkWeight->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    m_linkLayout->addRow(m_linkType);
    m_linkLayout->addRow(tr("Operator:"), m_linkOperation);
    m_linkLayout->addRow(tr("Weight:"), m_linkWeight);
    m_linkLayout->addRow(m_linkSecondary);

    m_linkFrame->setLayout(m_linkLayout);

    m_linkLayout->setSizeConstraint(QLayout::SetFixedSize);

    // By default, hide the frames and the buttons
    m_linkFrame->hide();
    m_boxFrame->hide();
    m_scriptFrame->hide();
    m_okBtn->hide();
    m_cancelBtn->hide();
}

PropertiesPanel::~PropertiesPanel()
{
    delete m_scriptFrame;
    delete m_scriptName;
    delete m_timeLabel;
    delete m_timeValue;
    delete m_timeUnit;
    delete m_boxName;
    delete m_boxOutputType;
    delete m_boxFrame;
    delete m_linkFrame;
    delete m_rowsInput;
    delete m_colsInput;
    delete m_saveActivity;
    delete m_okBtn;
    delete m_cancelBtn;
    delete m_linkType;
    delete m_linkOperation;
    delete m_linkSecondary;
    delete m_linkWeight;
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

QComboBox *PropertiesPanel::linkOperation() const
{
    return m_linkOperation;
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

QDoubleSpinBox *PropertiesPanel::timeValue() const
{
    return m_timeValue;
}

void PropertiesPanel::setTimeValue(QDoubleSpinBox *timeValue)
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

/**
 * @brief PropertiesPanel::displayBoxProperties updates the contents of the PropertiesPanel to
 * display the properties of the selected box
 */
void PropertiesPanel::displayBoxProperties(DiagramBox *box)
{
    if (box == NULL)
        informUserAndCrash(tr("Cannot display box's properties because box is null!"));

    // Hide the other frames
    m_linkFrame->hide();
    m_scriptFrame->hide();

    // Update the fields with the selected box
    m_boxName->setText(box->name());

    // Check the "save activity" box according to the box's flag
    m_saveActivity->setChecked(box->saveActivity());

    OutputType oType = box->outputType();

    if (oType == MATRIX) {
        m_boxOutputType->setText(tr("matrix"));
        m_rowsInput->setValue(box->rows());
        m_colsInput->setValue(box->cols());

        // Re-insert input for row and columns since it's a matrix
        QWidget *dimLabel = NULL;
        if (dimLabel = m_boxLayout->labelForField(m_colsInput))
            dimLabel->show();
        else
            informUserAndCrash(tr("Failed to fetch label for field 'columns'"));
        m_rowsInput->show();

        if (dimLabel = m_boxLayout->labelForField(m_rowsInput))
            dimLabel->show();
        else
            informUserAndCrash(tr("Failed to fetch label for field 'rows'"));
        m_colsInput->show();
    } else if (oType == SCALAR) {
        m_boxOutputType->setText(tr("scalar"));

        // Hide input for rows and columns since it's a scalar
        QWidget *dimLabel = NULL;
        if (dimLabel = m_boxLayout->labelForField(m_colsInput))
            dimLabel->hide();
        else
            informUserAndCrash(tr("Failed to fetch label for field 'columns'"));
        m_colsInput->hide();

        if (dimLabel = m_boxLayout->labelForField(m_rowsInput))
            m_rowsInput->hide();
        else
            informUserAndCrash(tr("Failed to fetch label for field 'rows'"));
        dimLabel->hide();
    } else {
        informUserAndCrash(tr("Unsupported output type for a box. Supported types are MATRIX and SCALAR"));
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
    m_boxFrame->hide();
    m_scriptFrame->hide();

    InputType linkType = link->to()->inputType();

    // Populate the panels with the link's properties
    m_linkType->setText(inputTypeToString(linkType));
    m_linkSecondary->setChecked(link->secondary());

    // Weight and operator are not applicable for SIMPLE_MATRIX type, so hide them
    if (linkType == SIMPLE_MATRIX) {
        QWidget *fieldLabel = NULL;
        if (fieldLabel = m_linkLayout->labelForField(m_linkWeight))
            fieldLabel->hide();
        else
            informUserAndCrash(tr("Failed to fetch label for field 'weight'"));
        m_linkWeight->hide();

        if (fieldLabel = m_linkLayout->labelForField(m_linkOperation))
            fieldLabel->hide();
        else
            informUserAndCrash(tr("Failed to fetch label for field 'operation'"));
        m_linkOperation->hide();
    } else {
        // Re-enable the weights and operator since they are applicable
        m_linkWeight->setValue(link->weight());

        QWidget *fieldLabel = NULL;
        if (fieldLabel = m_linkLayout->labelForField(m_linkWeight))
            fieldLabel->show();
        else
            informUserAndCrash(tr("Failed to fetch label for field 'weight'"));
        m_linkWeight->show();

        if (fieldLabel = m_linkLayout->labelForField(m_linkOperation))
            fieldLabel->show();
        else
            informUserAndCrash(tr("Failed to fetch label for field 'operation'"));
        m_linkOperation->setCurrentIndex(link->operation()); // Careful of order (see enum definition)
        m_linkOperation->show();
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
    m_boxFrame->hide();
    m_linkFrame->hide();

    // Populate fields
    m_scriptName->setText(script->name());
    m_timeValue->setValue(script->timeValue());
    m_timeUnit->setCurrentIndex(script->timeUnit());

    // Show the script's frame
    m_scriptFrame->show();
    m_okBtn->show();
    m_cancelBtn->show();
}

void PropertiesPanel::convertTimeValues(int unit)
{
    m_timeValue->setValue(1000.0 / m_timeValue->value());

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

/**
 * @brief PropertiesPanel::updateBoxProperties is called when the user clicked "OK" after changing
 * some properties of the selected box. It updates the selected box's properties based on the
 * contents of the fields in the properties panel
 * @param box
 */
void PropertiesPanel::updateBoxProperties(DiagramBox *box)
{
    if (box == NULL)
        informUserAndCrash(tr("Cannot update box's properties: box is null!"));

    // If the box's output is matrix, then set its rows and cols
    if (box->outputType() == MATRIX) {
        box->setRows(m_rowsInput->value());
        box->setCols(m_colsInput->value());
        // Make sure to call updateSizeIcon() BEFORE rescaleSvgItem() because the latter is based on the former
        updateSizeIcon(box);
        rescaleSvgItem(box->sizeIcon(),
                       QSizeF(box->bWidth() / 3 - 1.5, box->bHeight() - box->tHeight() - 2.5),
                       QPointF(2 * box->bWidth() / 3, 1.5));
    }

    // Set the box's "save activity" flag
    box->setSaveActivity(m_saveActivity->isChecked());
}

void PropertiesPanel::updateLinkProperties(Link *link)
{
    if (link == NULL)
        informUserAndCrash(tr("Cannot update link's properties: link is null!"));

    // If the link is not SIMPLE_MATRIX, set the weight and the operator
    if (link->to()->inputType() != SIMPLE_MATRIX) {
        link->setWeight(m_linkWeight->value());
        link->setOperation(m_linkOperation->currentData().value<LinkOperation>());
    }
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

    script->setTimeValue(m_timeValue->value());
    script->setTimeUnit(m_timeUnit->currentData().value<TimeUnit>());
}
