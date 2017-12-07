#include "function.h"

Function::Function() : QTreeWidgetItem()
{

}

QString Function::name() const
{
    return m_name;
}

void Function::setName(const QString &name)
{
    m_name = name;
}

std::vector<InputSlot> Function::inputs() const
{
    return m_inputs;
}

void Function::setInputs(const std::vector<InputSlot> &inputs)
{
    m_inputs = inputs;
}

OutputSlot Function::output() const
{
    return m_output;
}

void Function::setOutput(const OutputSlot &output)
{
    m_output = output;
}

/*
QIcon Function::icon() const
{
    return m_icon;
}

void Function::setIcon(const QIcon &value)
{
    m_icon = value;
}

//*/
