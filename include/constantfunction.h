#ifndef CONSTANTFUNCTION_H
#define CONSTANTFUNCTION_H

#include "function.h"
#include "types.h"

class ConstantFunction : public Function
{
public:
    ConstantFunction(const QString &name,
                     const QString &iconPath,
                     const QIcon &icon,
                     const OutputType outputType);
};

#endif // CONSTANTFUNCTION_H
