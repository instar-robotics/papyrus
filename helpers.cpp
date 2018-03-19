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
