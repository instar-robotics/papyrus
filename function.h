#ifndef FUNCTION_H
#define FUNCTION_H

#include "types.h"

#include <vector>

#include <QString>
#include <QIcon>
#include <QTreeWidgetItem>

/**
 * @brief The Function class describes a neural function and comes from parsing a valid XML
 * description file.
 */

class Function : public QTreeWidgetItem
{
public:
    Function();

    QString name() const;
    void setName(const QString &name);

    std::vector<InputSlot> inputs() const;
    void setInputs(const std::vector<InputSlot> &inputs);

    OutputSlot output() const;
    void setOutput(const OutputSlot &output);

//    QIcon icon() const;
//    void setIcon(const QIcon &value);

private:
    QString m_name;
//    QIcon m_icon;
    std::vector<InputSlot> m_inputs;
    OutputSlot m_output;
};

#endif // FUNCTION_H
