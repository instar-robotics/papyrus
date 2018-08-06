#include "librarypanel.h"
#include "constants.h"
#include "function.h"
#include "outputslot.h"
#include "inputslot.h"
#include "slot.h"

#include <iostream>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QDrag>
#include <QIcon>
#include <QDebug>

LibraryPanel::LibraryPanel(QWidget *parent) : QTreeWidget(parent)
{
    setDragEnabled(true);
    setColumnCount(1);        // Just the function's name (an icon is added)
    setHeaderHidden(true);    // Hide the header, we don't need it
    setAnimated(true);
    setIconSize(QSize(LIBRARY_ICON_SIZE, LIBRARY_ICON_SIZE));
    setIndentation(0);
    setRootIsDecorated(true); // Try to show the little arrow (doesn't work)
}

void LibraryPanel::dragEnterEvent(QDragEnterEvent *evt)
{
    std::cout << "Entering drag on library" << std::endl;
    evt->accept();
}

void LibraryPanel::dragMoveEvent(QDragMoveEvent *evt)
{
    std::cout << "Move drag on library" << std::endl;
    evt->accept();
}

void LibraryPanel::dropEvent(QDropEvent *evt)
{
    std::cout << "Drop on library" << std::endl;
    evt->accept();
}

void LibraryPanel::startDrag(Qt::DropActions)
{
    Function *item = static_cast<Function *>(currentItem());
    QString iconFilepath = item->iconFilepath();
    QIcon icon = item->icon(0);
    QString name = item->name();
    QString descriptionFile = item->descriptionFile();
    QString libname = item->libName();

    OutputSlot *outputSlot = item->output();
    OutputType outputType = outputSlot->outputType();
    bool constant = item->constant();

    std::vector<InputSlot *>inputSlots = item->inputs();
    int nbInputs = inputSlots.size();

    QByteArray itemData;
    QDataStream dataStream(&itemData, QIODevice::WriteOnly);

    dataStream << name << iconFilepath << icon << descriptionFile << (qint32)outputType << constant << nbInputs << libname;

    // Add all input slots' name, type and multiple
    foreach(InputSlot *i, inputSlots) {
        dataStream << i->name() << (qint32)i->inputType() << i->multiple();
    }

    QMimeData *mimeData = new QMimeData;
    mimeData->setData(LibraryPanel::libraryItemMimeType(), itemData);

    QDrag *drag = new QDrag(this);
    drag->setMimeData(mimeData);
    drag->setPixmap(icon.pixmap(QSize(LIBRARY_ICON_SIZE, LIBRARY_ICON_SIZE)));
    drag->exec(Qt::CopyAction);
}

