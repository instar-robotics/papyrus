#include "constantfunction.h"
#include "constants.h"

ConstantFunction::ConstantFunction(const QString &name,
                                   const QString &iconPath,
                                   const QIcon &icon,
                                   const OutputType outputType) :
    Function("")
{
    setName(name);
    setIconFilepath(iconPath);

    setText(0, m_name);
    setIcon(0, icon);
    setSizeHint(0, QSize(LIBRARY_ICON_SIZE, LIBRARY_ICON_SIZE));

    OutputSlot *outputSlot = new OutputSlot;
    outputSlot->setOutputType(outputType);
    m_output = outputSlot;
}
