#include "finddialog.h"
#include "diagramview.h"
#include "homepage.h"
#include "diagrambox.h"
#include "link.h"
#include "inputslot.h"

#include <QDebug>
#include <QRegularExpression>
#include <QRegularExpressionMatch>

FindDialog::FindDialog(QTabWidget *tabWidget, QWidget *parent)
    : QDialog(parent),
      m_infoLabel(tr("Type to find Functions or Links, enter names or UUID.")),
      m_legend(tr("Name or UUID:")),
      m_findNext(tr("Next")),
      m_findPrev(tr("Previous")),
      m_tabWidget(tabWidget),
      m_lastBtnClicked(nullptr),
      m_idx(0)
{
	setModal(false);
	setWindowTitle(tr("Find..."));

	m_layout.addWidget(&m_infoLabel, 0, 0, 1, 3);
	m_layout.addWidget(&m_legend, 1, 0);
	m_layout.addWidget(&m_input, 1, 1, 1, 2);
	m_layout.addWidget(&m_result, 2, 0);
	m_layout.addWidget(&m_findPrev, 2, 1);
	m_layout.addWidget(&m_findNext, 2, 2);

	m_result.clear();
	m_findPrev.setEnabled(false);
	m_findNext.setEnabled(false);

	setLayout(&m_layout);

	// Set initial scene
	updateCurrentScene(tabWidget->currentIndex());

	connect(tabWidget, SIGNAL(currentChanged(int)), this, SLOT(updateCurrentScene(int)));
	connect(&m_input, SIGNAL(textChanged(QString)), this, SLOT(find(QString)));
	connect(&m_findPrev, SIGNAL(clicked(bool)), this, SLOT(prev()));
	connect(&m_findNext, SIGNAL(clicked(bool)), this, SLOT(next()));
}

/**
 * @brief FindDialog::updateCurrentScene receives the new tab index of the tab widget when the user
 * switches scene by switching tabs. This is used to update the pointer to the current scene, so that
 * the find function works on the correct scene.
 * @param idx
 */
void FindDialog::updateCurrentScene(int idx)
{
	if (m_tabWidget == nullptr) {
		qWarning() << "[FindDialog::updateCurrentScene] null ptr instead of tab widget: cannot "
		              "update current scene.";
		return;
	}

	// Try to get the view associated with the new tab we're in
	DiagramView *dView = dynamic_cast<DiagramView *>(m_tabWidget->widget(idx));
	if (dView == nullptr) {
		// If we failed, check if this is because we are on the Home page, in which case, ignore
		if (dynamic_cast<HomePage *>(m_tabWidget->widget(idx)) != nullptr)
			return;

		qWarning() << "[FindDialog::updateCurrentScene] could not fetch new DiagramView: cannot "
		              "update current scene";
		return;
	}

	m_currentView = dView;
	m_currentScene = dynamic_cast<DiagramScene *>(dView->scene());

	if (m_currentScene == nullptr) {
		qWarning() << "[FindDialog::updateCurrentScene] could not get DiagramScene from DiagramView:"
		              " cannot update current scene!";
		return;
	}
}

/**
 * @brief FindDialog::find takes the user input and perform a search on the current active scene.
 * Input is analyzed: it this is recognized as a UUID, then we'll perform a UUID search, otherwise
 * we'll perform a search by Function name, Function title and Link's value.
 * @param request
 * @return
 */
QList<QGraphicsItem *> FindDialog::find(QString request)
{
	if (m_currentScene == nullptr) {
		qWarning() << "[FindDialog::find] could not find items: no current scene!";
		return QList<QGraphicsItem *>();
	}

	// Clear the result info text if no request, and disable buttons
	if (request.isEmpty()) {
		m_result.clear();
		m_findPrev.setEnabled(false);
		m_findNext.setEnabled(false);
	}

	// Reset the current index & last click button
	m_idx = 0;
	m_lastBtnClicked = nullptr;

	// Empty previous search
	m_matches.clear();

	// Build regex to detect UUID search
	QRegularExpression re("^{?([a-g0-9]{8})[-_]([a-g0-9]{4})[-_]([a-g0-9]{4})[-_]([a-g0-9]{4})[-_]([a-g0-9]{12})}?$");

	QRegularExpressionMatch match = re.match(request);
	QUuid targetUuid;
	if (match.hasMatch()) {
		targetUuid = QUuid(QString("{%1-%2-%3-%4-%5}")
		                 .arg(match.captured(1))
		                 .arg(match.captured(2))
		                 .arg(match.captured(3))
		                 .arg(match.captured(4))
		                 .arg(match.captured(5)));
	}

	// Traverse elements
	foreach (QGraphicsItem *item, m_currentScene->items()) {
		DiagramBox *box = dynamic_cast<DiagramBox *>(item);
		if (box != nullptr) {
			if (!targetUuid.isNull()) {
				if (box->uuid() == targetUuid)
					m_matches << box;
			} else {
				if (box->name().toLower().contains(request.toLower())
				    || box->title().toLower().contains(request.toLower()))
					m_matches << box;
			}
			continue;
		}

		Link *link = dynamic_cast<Link *>(item);
		if (link != nullptr) {
			if (!targetUuid.isNull()) {
				if (link->uuid() == targetUuid)
					m_matches << link;
			} else {
				if (link->isStringLink() && link->value().toLower().contains(request.toLower()))
					m_matches << link;
			}
			continue;
		}

		InputSlot *iSlot = dynamic_cast<InputSlot *>(item);
		if (iSlot != nullptr) {
			if (!targetUuid.isNull()) {
				if (iSlot->uuid() == targetUuid)
					m_matches << iSlot;
			} else {
				if (iSlot->name().toLower().contains(request.toLower()))
					m_matches << iSlot;
			}
			continue;
		}
	}

	m_findPrev.setEnabled(false);

	// Display result message
	if (!targetUuid.isNull()) {
		// Display the result message, normally we should not have more than one hit (it's a UUID)
		if (m_matches.size() == 0) {
			m_result.setText(tr("UUID not found."));
			m_findNext.setEnabled(false);
		} else if (m_matches.size() == 1) {
			m_result.setText(tr("UUID was found!"));
			m_findNext.setEnabled(true);
		} else {
			m_result.setText(QString("%1 matching UUIDs were found (conflict!)")
			                 .arg(m_matches.size()));
			m_findNext.setEnabled(true);
		}
	}
	else {
		if (m_matches.size() == 0) {
			m_result.setText(tr("No match."));
			m_findNext.setEnabled(false);
		} else if (m_matches.size() == 1) {
			m_result.setText(tr("1 match."));
			m_findNext.setEnabled(true);
		} else {
			m_result.setText(QString("%1 matches").arg(m_matches.size()));
			m_findNext.setEnabled(true);
		}
	}

	return m_matches;
}

void FindDialog::next()
{
	// Make sure we have a scene & view to center items
	if (m_currentScene == nullptr) {
		qWarning() << "[FindDialog::next] cannot find next item: no scene!";
		return;
	}

	if (m_currentView == nullptr) {
		qWarning() << "[FindDialog::next] cannot find next item: no view!";
		return;
	}

	// If we previously had clicked on previous, then increment m_idx before
	if (m_lastBtnClicked == &m_findPrev)
		// m_idx should not be able to be over m_matches.size()-1 here, but cap for sanity
		m_idx = qMin(m_idx + 1, m_matches.size() - 1);

	m_currentView->fitInView(m_matches.at(m_idx)->boundingRect().adjusted(-20, -20, 20, 20),
	                         Qt::KeepAspectRatio);
	m_matches.at(m_idx)->setSelected(true);
	m_currentView->centerOn(m_matches.at(m_idx));

	m_idx += 1;

	// Disable 'next' button if we reached last element, and cap m_idx
	if (m_idx >= m_matches.size()) {
		m_findNext.setEnabled(false);
		m_idx = m_matches.size() - 1;
	} else {
		m_findNext.setEnabled(true);
	}

	// Enable 'prev' button if we have more than one element
	if (m_idx > 1)
		m_findPrev.setEnabled(true);
	else
		m_findPrev.setEnabled(false);

	// Set the last clicked button
	m_lastBtnClicked = &m_findNext;
}

void FindDialog::prev()
{
	// Make sure we have a scene & view to center items
	if (m_currentScene == nullptr) {
		qWarning() << "[FindDialog::prev] cannot find previous item: no scene!";
		return;
	}

	if (m_currentView == nullptr) {
		qWarning() << "[FindDialog::prev] cannot find previous item: no view!";
		return;
	}

	// If we previously had clicked on next, then decrement m_idx before
	if (m_lastBtnClicked == &m_findNext && m_idx != m_matches.size() - 1)
		// m_idx should not be able to be under 0 here, but cap for sanity
		m_idx = qMax(m_idx - 1, 0);

	m_idx -= 1;

	// Disable 'prev' button if we reached first element
	if (m_idx <= 0) {
		m_idx = 0;
		m_findPrev.setEnabled(false);
	} else {
		m_findPrev.setEnabled(true);
	}

	m_currentView->fitInView(m_matches.at(m_idx)->boundingRect().adjusted(-20, -20, 20, 20),
	                         Qt::KeepAspectRatio);
	m_currentView->centerOn(m_matches.at(m_idx));
	m_matches.at(m_idx)->setSelected(true);


	// Enable 'next' button if we have more than one element
	if (m_matches.size() > 1)
		m_findNext.setEnabled(true);
	else
		m_findNext.setEnabled(false);

	// Update last button clicked
	m_lastBtnClicked = &m_findPrev;
}
