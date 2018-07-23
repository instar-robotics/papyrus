#ifndef HOMEPAGE_H
#define HOMEPAGE_H

#include <QFrame>
#include <QLabel>
#include <QComboBox>

class HomePage : public QFrame
{
    Q_OBJECT
public:
    explicit HomePage(QWidget *parent = nullptr);

    QLabel *rosMasterStatus() const;
    void setRosMasterStatus(QLabel *rosMasterStatus);

private:
    QLabel *m_title;
    QLabel *m_rosMasterStatus;
    QLabel *m_kNodesLabel;
    QComboBox *m_kheopsNodes;

signals:

public slots:
    void onRosMasterChange(bool isOnline);
};

#endif // HOMEPAGE_H
