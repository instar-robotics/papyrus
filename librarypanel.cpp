#include "librarypanel.h"
#include "constants.h"
#include <iostream>

#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QDrag>
#include <QIcon>

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

void LibraryPanel::startDrag(Qt::DropActions supportedActions)
{
    std::cout << "Start drag on library" << std::endl;

    QTreeWidgetItem *item = currentItem();
    QIcon icon = item->icon(0);
    QString name = item->text(0);

    QByteArray itemData;
    QDataStream dataStream(&itemData, QIODevice::WriteOnly);

    dataStream << name << icon;

    QMimeData *mimeData = new QMimeData;
    mimeData->setData(LibraryPanel::libraryItemMimeType(), itemData);

    QDrag *drag = new QDrag(this);
    drag->setMimeData(mimeData);
    drag->setPixmap(icon.pixmap(QSize(LIBRARY_ICON_SIZE, LIBRARY_ICON_SIZE)));
    std::cout << "End of drag: " << drag->exec(Qt::CopyAction) << std::endl;
}

