#ifndef PROPERTIESPANEL_H
#define PROPERTIESPANEL_H

#include "diagrambox.h"
#include "link.h"

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
    ~PropertiesPanel();

    void updateBoxProperties(DiagramBox *box);
    void updateLinkProperties(Link *link);

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

private:
    QFormLayout *m_boxLayout;  // Layout for the box properties (access needed to hide rows)
    QFrame *m_boxFrame;        // Container for box's properties
    QLabel *m_boxName;         // Display the name of the box
    QLabel *m_boxOutputType;   // Display the box's output type (scalar, matrix)
    QSpinBox *m_rowsInput;     // Spin box to input number of rows in the output (if matrix)
    QSpinBox *m_colsInput;     // Spin box to input number of columns in the output (if matrix)
    QCheckBox *m_saveActivity; // To enable saving the activity of the box
    QPushButton *m_okBtn;      // Button used to validate changes in parameters
    QPushButton *m_cancelBtn;  // Button used to discard changes in parameters and restore current

    QFormLayout *m_linkLayout;   // Layout for the link properties (access needed to hide rows)
    QFrame *m_linkFrame;         // Container for link's properties
    QLabel *m_linkType;          // Display the type of the link
    QComboBox *m_linkOperation;     // Chose the operation between inputs and weights
    QCheckBox *m_linkSecondary;  // Will display if the link is secondary or not
    QDoubleSpinBox *m_linkWeight; // Spin box to set the weight of the link

public slots:
    void displayBoxProperties(DiagramBox *box);
    void displayLinkProperties(Link *link);
};

#endif // PROPERTIESPANEL_H
