#ifndef PROPERTIESPANEL_H
#define PROPERTIESPANEL_H

#include "diagrambox.h"

#include <QGroupBox>
#include <QLineEdit>
#include <QFrame>
#include <QLabel>
#include <QSpinBox>
#include <QPushButton>

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

private:
    QFrame *m_boxFrame;        // Container for box's properties
    QLabel *m_boxName;         // Display the name of the box
    QLabel *m_boxOutputType;   // Display the box's output type (scalar, matrix)
    QFrame *m_linkFrame;       // Container for link's properties
    QSpinBox *m_rowsInput;     // Spin box to input number of rows in the output (if matrix)
    QSpinBox *m_colsInput;     // Spin box to input number of columns in the output (if matrix)
    QPushButton *m_okBtn;      // Button used to validate changes in parameters
    QPushButton *m_cancelBtn;  // Button used to discard changes in parameters and restore current

public slots:
    void displayBoxProperties(DiagramBox *box);
    void displayLinkProperties();
};

#endif // PROPERTIESPANEL_H
