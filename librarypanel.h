#ifndef LIBRARYPANEL_H
#define LIBRARYPANEL_H

#include <QTreeWidget>

class LibraryPanel : public QTreeWidget
{
    Q_OBJECT

public:
    explicit LibraryPanel(QWidget *parent = 0);

    static QString libraryItemMimeType() { return QStringLiteral("application/x-neural-box"); }

protected:
    void dragEnterEvent(QDragEnterEvent *evt) override;
    void dragMoveEvent(QDragMoveEvent *evt) override;
    void dropEvent(QDropEvent *evt) override;
    void startDrag(Qt::DropActions supportedActions) override;
};

#endif // LIBRARYPANEL_H