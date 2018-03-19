#include "propertiespanel.h"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QDebug>
#include <QLabel>

PropertiesPanel::PropertiesPanel(QWidget *parent) : QGroupBox(parent),
                                                    m_boxFrame(NULL),
                                                    m_boxName(NULL),
                                                    m_boxOutputType(NULL),
                                                    m_linkFrame(NULL),
                                                    m_rowsInput(NULL),
                                                    m_colsInput(NULL),
                                                    m_okBtn(NULL),
                                                    m_cancelBtn(NULL)
{
    setTitle(tr("Properties"));

    QVBoxLayout *layout = new QVBoxLayout;
    m_boxFrame = new QFrame;
    m_linkFrame = new QFrame;
    m_okBtn = new QPushButton("OK");
    m_okBtn->setIcon(QIcon(":/icons/icons/check.svg"));
    m_okBtn->setToolTip(tr("Apply changes to the selected item"));
    m_cancelBtn = new QPushButton("Cancel");
    m_cancelBtn->setIcon(QIcon(":/icons/icons/cancel.svg"));
    m_cancelBtn->setIconSize(QSize(22, 22)); // Shrink the cancel icon size a little
    m_cancelBtn->setToolTip(tr("Discard changes and reset to the selected item's"));
    layout->addWidget(m_boxFrame);
    layout->addWidget(m_linkFrame);
    QHBoxLayout *btnsLayout = new QHBoxLayout;
    btnsLayout->addWidget(m_okBtn);
    btnsLayout->addWidget(m_cancelBtn);
    layout->addLayout(btnsLayout);
    setLayout(layout);

    // Create the layout for the two frames
    QFormLayout *boxLayout = new QFormLayout;
    m_boxName = new QLabel;
    m_boxOutputType = new QLabel;
    m_rowsInput = new QSpinBox;
    m_colsInput = new QSpinBox;

    // Parameterize the fields
    boxLayout->setContentsMargins(0, 0, 0, 0); // Reduce inner margins due to lack of space
    m_boxName->setAlignment(Qt::AlignCenter);

    // Add the fields to the layout
    boxLayout->addRow(m_boxName);
    boxLayout->addRow(tr("Type:"), m_boxOutputType);
    boxLayout->addRow(tr("Rows:"), m_rowsInput);
    boxLayout->addRow(tr("Cols:"), m_colsInput);

    m_boxFrame->setLayout(boxLayout);

    m_linkFrame->setLayout(new QVBoxLayout);

    // By default, hide the frames and the buttons
    m_linkFrame->hide();
    m_boxFrame->hide();
    m_okBtn->hide();
    m_cancelBtn->hide();
}

PropertiesPanel::~PropertiesPanel()
{
    delete m_boxName;
    delete m_boxOutputType;
    delete m_boxFrame;
    delete m_linkFrame;
    delete m_rowsInput;
    delete m_colsInput;
    delete m_okBtn;
    delete m_cancelBtn;
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

/**
 * @brief PropertiesPanel::displayBoxProperties updates the contents of the PropertiesPanel to
 * display the properties of the selected box
 */
void PropertiesPanel::displayBoxProperties(DiagramBox *box)
{
    if (box == NULL)
        qFatal("Cannot display box's properties because box is null!");

    // Hide the link frame
    m_linkFrame->hide();

    // Update the fields with the selected box
    m_boxName->setText("<b>" + box->name() + "</b>");

    OutputType oType = box->outputType();

    if (oType == MATRIX) {
        m_boxOutputType->setText(tr("matrix"));
        m_rowsInput->setValue(box->rows());
        m_colsInput->setValue(box->cols());
        // Re-enable input for row and columns since it's a matrix
        m_colsInput->setEnabled(true);
        m_rowsInput->setEnabled(true);
    } else if (oType == SCALAR) {
        m_boxOutputType->setText(tr("scalar"));
        // Disable input for rows and columns since it's a scalar
        m_colsInput->setEnabled(false);
        m_rowsInput->setEnabled(false);
    } else {
        qFatal("Output type not supported");
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
void PropertiesPanel::displayLinkProperties()
{
    // Hide the box frame
    m_boxFrame->hide();

    // Show the link frame
    m_linkFrame->show();
}
