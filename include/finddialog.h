#ifndef FINDDIALOG_H
#define FINDDIALOG_H

#include "diagramscene.h"
#include "diagramview.h"
#include "diagrambox.h"
#include "link.h"

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QGridLayout>
#include <QTabWidget>
#include <QUuid>

class FindDialog : public QDialog
{
	Q_OBJECT

public:
	explicit FindDialog(QTabWidget *tabWidget, QWidget *parent = nullptr);

private:
	QLabel m_infoLabel;
	QLabel m_legend;
	QLineEdit m_input;
	QLabel m_result;
	QPushButton m_findNext;
	QPushButton m_findPrev;

	QGridLayout m_layout;

	QTabWidget *m_tabWidget;

	DiagramScene *m_currentScene;
	DiagramView *m_currentView;

	QList<QGraphicsItem *> m_matches;
	QPushButton *m_lastBtnClicked; // used to know when we switch click from previous to next to prevent displaying twice

	int m_idx; // the index in the list of matches we are currently centered on

private slots:
	void updateCurrentScene(int idx);
	QList<QGraphicsItem *> find(QString request);
	void next();
	void prev();
};

#endif // FINDDIALOG_H
