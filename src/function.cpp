#include "function.h"

Function::Function(const QString &path) : QTreeWidgetItem(),
                                          m_descriptionFile(path),
                                          m_constant(false)

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

std::vector<InputSlot *> Function::inputs() const
{
    return m_inputs;
}

void Function::addInputSlot(InputSlot *slot)
{
    m_inputs.push_back(slot);
}

OutputSlot* Function::output() const
{
    return m_output;
}

void Function::setOutput(OutputSlot *output)
{
    m_output = output;
}

QString Function::descriptionFile() const
{
    return m_descriptionFile;
}

bool Function::constant() const
{
    return m_constant;
}

void Function::setConstant(bool constant)
{
    m_constant = constant;
}

QString Function::libName() const
{
    return m_libName;
}

void Function::setLibName(const QString &libName)
{
    m_libName = libName;
}

QString Function::iconFilepath() const
{
    return m_iconFilepath;
}

void Function::setIconFilepath(const QString &value)
{
    m_iconFilepath = value;
}
