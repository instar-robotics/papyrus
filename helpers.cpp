#include "helpers.h"

/**
 * @brief getMainWindow returns the instance of the main Papyrus window, checking that is exists
 * @return
 */
PapyrusWindow *getMainWindow()
{
    PapyrusWindow *mainWindow = NULL;

    foreach (QWidget *w, qApp->topLevelWidgets()) {
        if (PapyrusWindow *mW = qobject_cast<PapyrusWindow *>(w)) {
            mainWindow = mW;
            break;
        }
    }

    if (mainWindow == NULL)
        qFatal("Impossible to fetch the main window!");

    return mainWindow;
}

/**
 * @brief outputTypeToString converts a value from the enum OutputType to a QString, used to save
 * script for instance
 * @param outputType the value to convert
 * @return
 */
QString outputTypeToString(OutputType outputType)
{
    switch (outputType) {
    case SCALAR:
        return QString("Scalar");
        break;
    case MATRIX:
        return QString("Matrix");
    default:
        qFatal("Unsupported OutputType when converting to QString.");
        break;
    }
}

/**
 * @brief inputTypeToString converts a value from the enum InputType to a QString, used to save
 * script for instance
 * @param inputType the value to convert
 * @return
 */
QString inputTypeToString(InputType inputType)
{
    switch (inputType) {
    case SCALAR_SCALAR:
        return QString("SCALAR_SCALAR");
        break;
    case SIMPLE_MATRIX:
        return QString("SIMPLE_MATRIX");
        break;
    case SCALAR_MATRIX:
        return QString("SCALAR_MATRIX");
        break;
    case MATRIX_MATRIX:
        return QString("MATRIX_MATRIX");
        break;
    default:
        qFatal("Unsupported InputType when converting to QString.");
        break;
    }
}
