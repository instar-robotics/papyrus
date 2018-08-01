#include "nodeschooser.h"
#include "ui_nodeschooser.h"

#include "helpers.h"

NodesChooser::NodesChooser(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::NodesChooser)
{
    ui->setupUi(this);

    populateKheopsNodes();

    // Make sure the first one is selected
    ui->comboBox->setCurrentIndex(0);

    m_selectedNode = ui->comboBox->currentText();
}

NodesChooser::~NodesChooser()
{
    delete ui;
}

void NodesChooser::populateKheopsNodes()
{
    // Populate the list of nodes upon creation
    QList<QString> kheopsNodes = getKheopsNodes();

    // Clear the list, add the "Current Script" first and populate
    ui->comboBox->clear();
    ui->comboBox->addItem(tr("Current Script"), tr("Current Script"));

    foreach (QString kheopsNode, kheopsNodes) {
        ui->comboBox->addItem(kheopsNode, kheopsNode);
    }
}

void NodesChooser::on_pushButton_clicked()
{
    ui->pushButton->setDisabled(true);
    populateKheopsNodes();
    ui->pushButton->setDisabled(false);
}

QString NodesChooser::selectedNode() const
{
    return m_selectedNode;
}

void NodesChooser::on_comboBox_currentTextChanged(const QString &arg1)
{
    m_selectedNode = arg1;
}
