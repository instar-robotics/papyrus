#ifndef NODESCHOOSER_H
#define NODESCHOOSER_H

#include <QDialog>
#include <QDebug>
#include <QString>

#include <ros/ros.h>

namespace Ui {
class NodesChooser;
}

class NodesChooser : public QDialog
{
    Q_OBJECT

public:
    explicit NodesChooser(QWidget *parent = 0);
    ~NodesChooser();

    void populateKheopsNodes();

    QString selectedNode() const;

private slots:
    void on_pushButton_clicked();

    void on_comboBox_currentTextChanged(const QString &arg1);

private:
    Ui::NodesChooser *ui;
    QString m_selectedNode;
};

#endif // NODESCHOOSER_H
