#include "variablewindow.h"
#include "ui_variablewindow.h"
#include "helpers.h"

#include <QScreen>
#include <QDebug>

VariableWindow::VariableWindow(QWidget *parent) :
    QDialog(parent),
    m_ui(new Ui::VariableWindow)
{
	m_ui->setupUi(this);

	// Resize window
	QScreen *screen = QGuiApplication::primaryScreen();
	QSize screenSize = screen->availableSize();
	resize(screenSize.width() / 2.0,
	       screenSize.height() / 2.0);

	m_ui->tableWidget->setColumnCount(3);
	m_ui->tableWidget->setHorizontalHeaderLabels(QStringList() << "Variable"
	                                             << "Value" << "Description");

	// Hide variables numbers
	m_ui->tableWidget->verticalHeader()->hide();
	m_ui->tableWidget->horizontalHeader()->setStretchLastSection(true);

	m_ui->pushButton->setText(tr("Add variable"));
	m_ui->pushButton->setIcon(QIcon(":/icons/icons/add.svg"));

	connect(m_ui->pushButton, SIGNAL(clicked(bool)), this, SLOT(onAddVariableClicked()));
	connect(m_ui->tableWidget, SIGNAL(cellChanged(int,int)), this, SLOT(onCellChanged(int,int)));
}

VariableWindow::~VariableWindow()
{
	delete m_ui;
}

void VariableWindow::setScript(Script *script)
{
	if (script == nullptr)
		return;

	m_script = script;

	// Update title
	QString t = QString("Variables for %1").arg(m_script->name());
	m_ui->label->setText(t);
	setWindowTitle(t);

	m_ui->tableWidget->clear();

	m_ui->tableWidget->setRowCount(1 + script->variables().size());

	int i = 0;
	foreach(QString var, m_script->variables().keys()) {
		QString value = m_script->variables()[var].first;
		QString desc = m_script->variables()[var].second;

		QTableWidgetItem *itemVar = new QTableWidgetItem(var);
		QTableWidgetItem *itemValue = new QTableWidgetItem(value);
		QTableWidgetItem *itemDesc = new QTableWidgetItem(desc);
		m_ui->tableWidget->setItem(i, 0, itemVar);
		m_ui->tableWidget->setItem(i, 1, itemValue);
		m_ui->tableWidget->setItem(i, 2, itemDesc);

		i += 1;
	}
}

QTableWidget *VariableWindow::getTabWidget()
{
	return m_ui->tableWidget;
}

Script *VariableWindow::script() const
{
	return m_script;
}

void VariableWindow::onAddVariableClicked()
{
	int nbRows = m_ui->tableWidget->rowCount();

	bool lastRowEmpty = m_ui->tableWidget->item(nbRows - 1, 0) == nullptr
	                    && m_ui->tableWidget->item(nbRows - 1, 1) == nullptr
	                    && m_ui->tableWidget->item(nbRows - 1, 2) == nullptr;

	// Don't allow adding variable if there's an empty row
	if (lastRowEmpty)
		return;

	m_ui->tableWidget->setRowCount(nbRows + 1);
}

/**
 * @brief VariableWindow::onCellChanged is used to make sure the user only use valid characters for
 * variable names
 * @param row
 * @param col
 */
void VariableWindow::onCellChanged(int row, int col)
{
	QTableWidgetItem *item = m_ui->tableWidget->item(row, col);

	if (item == nullptr)
		return;

	// Only check first column (variable name)
	if (col != 0)
		return;

	// This might be a bit violet, but replace the variable content with its sanitized version
	QString sanitizedName = sanitizeVariableName(item->text());

	// Only update if this is different (to prevent cursor jump)
	if (sanitizedName != item->text())
		item->setText(sanitizedName);
}
