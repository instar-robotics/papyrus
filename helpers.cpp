#include "helpers.h"

#include <QMessageBox>

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
 * @brief stringToOutputType converts a QString to the enum OutputType. It is case-insensitive
 * @param str the string to convert
 * @return
 */
OutputType stringToOutputType(QString str)
{
    QString lower = str.toLower();

    if (lower == "scalar")
        return SCALAR;

    if (lower == "matrix")
        return MATRIX;

    qFatal("Failed to parse string into OutputType");
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

/**
 * @brief stringToInputType converts a string to the enum InputType. It is case-insensitive.
 * @param str the string to convert
 * @return
 */
InputType stringToInputType(QString str)
{
    QString lower = str.toLower();

    if (lower == "scalar_scalar")
        return SCALAR_SCALAR;

    if (lower == "simple_matrix")
        return SIMPLE_MATRIX;

    if (lower == "scalar_matrix")
        return SCALAR_MATRIX;

    if (lower == "matrix_matrix")
        return MATRIX_MATRIX;

    qFatal("Failed to parse string to InputType");
}

/**
 * @brief canLink retursn whether the output type 'from' is compatible with input type 'to' in order
 * to create a link
 * @param from the @OutputType that we want to connect
 * @param to the @InputType we want to connect to
 * @return whether the connection is valid or not
 */
bool canLink(OutputType from, InputType to)
{
    // If the input slot expects a scalar, only a scalar can be connected to it
    if (from == SCALAR && to == SCALAR_SCALAR)
        return true;

    // If the input slot expects a matrix with any kind of connectivity, then only a matrix can
    // be connected
    if (from == MATRIX && to != SCALAR_SCALAR)
        return true;

    // other cases are invalid
    return false;
}

/**
 * @brief informUserAndCrash is used whenever we perform a check (for instance check if a pointer is
 * null before using it (that's good practise BTW!)) and we then want to crash the application.
 * Rather that only writing a debug message in the logs (which the user probably would not read, and
 * this he will only pester agains the application, wondering what happened), the developper MUST
 * use this function with an informative error message.
 * This error message will be displayed on a blocking, modal window, with a unique button "close",
 * then it calls qFatal() which makes the application crash.
 * This way, the user knows why it crashed, and it's also logged.
 *
 * TL;DR: use this function *everywhere* you want to make the program crash
 * @param text the informative error message that will be displayed to the user and the logs
 * @param title (optional) the title of the modal window, will default to "Papyrus is about to crash!"
 */
void informUserAndCrash(const QString &text, const QString &title)
{
    QMessageBox::critical(NULL, title, text, QMessageBox::Close);
    qFatal("[Papyrus] FATAL: %s", text.toStdString().c_str());
}
