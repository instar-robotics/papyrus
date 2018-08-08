#include "propertiespanel.h"
#include "helpers.h"
#include "constants.h"
#include "connectivitywindow.h"

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
                                                    m_linkConnectivityBtn(NULL),
                                                    m_linkConnectivityLabel(NULL),
                                                    m_linkConnectivity(NULL),
                                                    m_okBtn(NULL),
                                                    m_cancelBtn(NULL)
{
    setTitle(tr("Properties"));
    QVBoxLayout *m_panelLayout = new QVBoxLayout;
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
    m_panelLayout->addWidget(m_scriptFrame);
    m_panelLayout->addWidget(m_boxFrame);
    m_panelLayout->addWidget(m_linkFrame);
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
    m_timeValue = new QDoubleSpinBox;
    m_timeUnit = new QComboBox;
    m_encrypt = new QCheckBox;

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
    scriptLayout->addRow(tr("Encrypted:"), m_encrypt);

    m_scriptFrame->setLayout(scriptLayout);

    // Create the layout for the frames
    m_boxLayout = new QFormLayout;
    m_boxName = new QLabel;
    m_boxOutputType = new QLabel;
    m_rowsInput = new QSpinBox;
    m_colsInput = new QSpinBox;
    m_saveActivity = new QCheckBox(tr("Save Activity"));
    m_publish = new QCheckBox(tr("Publish output"));
    m_topic = new QLineEdit;

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
    connect(m_publish, SIGNAL(toggled(bool)), this, SLOT(toggleTopic(bool)));

    // Add the fields to the layout
    m_boxLayout->addRow(m_boxName);
    m_boxLayout->addRow(tr("Type:"), m_boxOutputType);
    m_boxLayout->addRow(tr("Rows:"), m_rowsInput);
    m_boxLayout->addRow(tr("Cols:"), m_colsInput);
    m_boxLayout->addRow(m_saveActivity);
    m_boxLayout->addRow(m_publish);
    m_boxLayout->addRow(tr("Topic:"), m_topic);

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
    m_linkWeight->setDecimals(6);
    m_linkWeight->setFixedWidth(150);
    m_linkWeight->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_linkValue = new QLineEdit;
    m_linkConnectivityBtn = new QPushButton(tr("Edit Connectivity"));
    m_linkConnectivityLabel = new QLabel(tr("Connectivity"));
    m_linkConnectivity = new QComboBox;
    m_linkConnectivity->addItem(tr("One to One"), ONE_TO_ONE);
    m_linkConnectivity->addItem(tr("One to All"), ONE_TO_ALL);
    m_linkConnectivity->addItem(tr("One to Neighboors"), ONE_TO_NEI);

    m_linkLayout->addRow(m_linkType);
    m_linkLayout->addRow(tr("Weight:"), m_linkWeight);
    m_linkLayout->addRow(tr("Value:"), m_linkValue);
    m_linkLayout->addRow(m_linkSecondary);
    m_linkLayout->addRow(m_linkConnectivityBtn);
    m_linkLayout->addRow(m_linkConnectivityLabel, m_linkConnectivity);

    m_linkFrame->setLayout(m_linkLayout);

    m_linkLayout->setSizeConstraint(QLayout::SetFixedSize);

    // By default, hide the frames and the buttons
    m_linkFrame->hide();
    m_boxFrame->hide();
    m_scriptFrame->hide();
    m_okBtn->hide();
    m_cancelBtn->hide();
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

QLineEdit *PropertiesPanel::linkValue() const
{
    return m_linkValue;
}

void PropertiesPanel::setLinkValue(QLineEdit *linkValue)
{
    m_linkValue = linkValue;
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

    // Check the "publish output" box according to the box's flag
    m_publish->setChecked(box->publish());

    // Get the topic name and enable / disable it based on the publish flag
    m_topic->setText(box->topic());
    m_topic->setEnabled(box->publish());

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
    } else if (oType == STRING) {
        m_boxOutputType->setText(tr("string"));

        // Hide input for rows and columns since it's not a matrix
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

    // Display "Edit Connectivity" button is link is SPARSE_MATRIX
    m_linkConnectivityBtn->disconnect(m_conn);

    if (linkType == SPARSE_MATRIX) {
        m_linkConnectivityBtn->setVisible(true);
        m_inputSize.setWidth(link->from()->box()->rows());
        m_inputSize.setHeight(link->from()->box()->cols());
        m_outputSize.setWidth(link->to()->box()->rows());
        m_outputSize.setHeight(link->to()->box()->cols());

        m_conn = connect(m_linkConnectivityBtn, SIGNAL(clicked(bool)),
                this, SLOT(showConnectivityWindow()));
    }
    else
        m_linkConnectivityBtn->setVisible(false);

    // Display the connectivity button choice for links of type "MATRIX_MATRIX"
    if (linkType == MATRIX_MATRIX) {
        m_linkConnectivity->setCurrentIndex(link->connectivity());
        m_linkConnectivityLabel->setVisible(true);
        m_linkConnectivity->setVisible(true);
    } else {
        m_linkConnectivityLabel->setVisible(false);
        m_linkConnectivity->setVisible(false);
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
    m_encrypt->setChecked(script->encrypt());

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

void PropertiesPanel::toggleTopic(bool isChecked)
{
    m_topic->setEnabled(isChecked);
}

void PropertiesPanel::showConnectivityWindow()
{
    ConnectivityWindow *connWin = new ConnectivityWindow(m_inputSize, m_outputSize);
    connWin->setWindowFlag(Qt::Dialog);
    connWin->setWindowModality(Qt::ApplicationModal);

    QScreen *primaryScreen = QGuiApplication::primaryScreen();
    if (primaryScreen == NULL)
        qFatal("No screen detected!");

    connWin->resize(0.8 * primaryScreen->availableSize());
    connWin->show();
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

    // Set the box's publish and topic name
    box->setPublish(m_publish->isChecked());
    QString topic = m_topic->text();
    if (topic.isEmpty())
        box->setTopic(box->uuid().toString());
    else
        box->setTopic(topic);
}

void PropertiesPanel::updateLinkProperties(Link *link)
{
    if (link == NULL)
        informUserAndCrash(tr("Cannot update link's properties: link is null!"));

    // Update either the weight or the value based on the link being a string link or not
    if (link->isStringLink())
        link->setValue(m_linkValue->text());
    else
        link->setWeight(m_linkWeight->value());

    link->setSecondary(m_linkSecondary->isChecked());

    // If the link is of type MATRIX_MATRIX, update the Connectivity
    if (link->to()->inputType() == MATRIX_MATRIX) {
        link->setConnectivity(m_linkConnectivity->currentData().value<Connectivity>());
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
    script->setEncrypt(m_encrypt->isChecked());
}
