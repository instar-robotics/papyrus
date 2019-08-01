#ifndef VARIABLEWINDOW_H
#define VARIABLEWINDOW_H

#include "script.h"

#include <QDialog>
#include <QTableWidgetItem>

namespace Ui {
class VariableWindow;
}

class VariableWindow : public QDialog
{
	Q_OBJECT

public:
	explicit VariableWindow(QWidget *parent = nullptr);
	~VariableWindow();

	Script *script() const;
	void setScript(Script *script);

	QTableWidget *getTabWidget();

private:
	Ui::VariableWindow *m_ui;

	Script *m_script;

private slots:
	void onAddVariableClicked();
	void onCellChanged(int row, int col);
};

#endif // VARIABLEWINDOW_H
