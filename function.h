#ifndef FUNCTION_H
#define FUNCTION_H

#include "outputslot.h"
#include "inputslot.h"

#include <vector>

#include <QString>
#include <QIcon>
#include <QTreeWidgetItem>

/**
 * @brief The Function class describes a neural function for the @Library and @LibraryPanel
 * and comes from parsing a valid XML description file. Basically, the @Category class
 * holds several @Function.
 * The @Function class is only meant to be stored inside the @Library, but the actual neural
 * function is created as a @DiagramBox from this lightweight @Function object when it is
 * dropped on the @DiagramScene.
 */

class Function : public QTreeWidgetItem
{
public:
    Function(QString &path);

    QString name() const;
    void setName(const QString &name);

    std::vector<InputSlot *> inputs() const;

    void addInputSlot (InputSlot *slot);

    OutputSlot *output() const;
    void setOutput(OutputSlot *output);

//    QIcon icon() const;
//    void setIcon(const QIcon &value);

    QString descriptionFile() const;

    bool constant() const;
    void setConstant(bool constant);

private:
    QString m_name;
    QString m_descriptionFile;
//    QIcon m_icon;
    std::vector<InputSlot *> m_inputs;
    OutputSlot *m_output;
    bool m_constant; // Indicae whether this represents a constant input
};

#endif // FUNCTION_H
