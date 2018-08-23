#ifndef PROPERTIESPANEL_H
#define PROPERTIESPANEL_H

#include "diagrambox.h"
#include "link.h"
#include "script.h"

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

    void updateBoxProperties(DiagramBox *box);
    void updateLinkProperties(Link *link);
    void updateScriptProperties(Script *script);

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

    QDoubleSpinBox *timeValue() const;
    void setTimeValue(QDoubleSpinBox *timeValue);

    QComboBox *timeUnit() const;
    void setTimeUnit(QComboBox *timeUnit);

    QLineEdit *linkValue() const;
    void setLinkValue(QLineEdit *linkValue);

    QPushButton *displayVisu() const;
    void setDisplayVisu(QPushButton *displayVisu);

private:
    QVBoxLayout *m_panelLayout;  // The properties panel's main layout
    QFrame *m_scriptFrame;       // Container for script's properties
    QLabel *m_scriptName;        // Label used to change the script (and tab) name
    QLabel *m_timeLabel;         // Contains either "frequency" or "period"
    QDoubleSpinBox *m_timeValue; // Used to input the script's frequency (or period)
    QComboBox *m_timeUnit;       // Used to select the unit (in Hz or ms)
    QCheckBox *m_encrypt;        // Whether or not the file is encrypted on save

    QFormLayout *m_boxLayout;  // Layout for the box properties (access needed to hide rows)
    QFrame *m_boxFrame;        // Container for box's properties
    QLabel *m_boxName;         // Display the name of the box
    QLabel *m_boxOutputType;   // Display the box's output type (scalar, matrix)
    QSpinBox *m_rowsInput;     // Spin box to input number of rows in the output (if matrix)
    QSpinBox *m_colsInput;     // Spin box to input number of columns in the output (if matrix)
    QCheckBox *m_saveActivity; // To enable saving the activity of the box
    QCheckBox *m_publish;      // To enable publish the output of the function
    QLineEdit *m_topic;        // To input the topic name for publishing
    QPushButton *m_displayVisu; // (TEMP) display the box's data vizualisation

    QFormLayout *m_linkLayout;    // Layout for the link properties (access needed to hide rows)
    QFrame *m_linkFrame;          // Container for link's properties
    QLabel *m_linkType;           // Display the type of the link
    QCheckBox *m_linkSecondary;   // Will display if the link is secondary or not
    QDoubleSpinBox *m_linkWeight; // Spin box to set the weight of the link
    QLineEdit *m_linkValue;       // Text field to enter the link's value (for string links)
    QPushButton *m_linkConnectivityBtn; // Button to launch the window that allows setting the connectivity
    QLabel *m_linkConnectivityLabel;
    QComboBox *m_linkConnectivity;
    QMetaObject::Connection m_conn;     // Contains the the Qt's connection object for the above button

    // Will contain the sizes of connected matrix when the link is SPARSE_MATRIX
    QSize m_inputSize;
    QSize m_outputSize;

    QPushButton *m_okBtn;      // Button used to validate changes in parameters
    QPushButton *m_cancelBtn;  // Button used to discard changes in parameters and restore current

public slots:
    void displayBoxProperties(DiagramBox *box);
    void displayLinkProperties(Link *link);
    void displayScriptProperties(Script *script);
    void convertTimeValues(int unit);
    void toggleTopic(bool isChecked);

private slots:
    void showConnectivityWindow();
};

#endif // PROPERTIESPANEL_H
