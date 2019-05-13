#include "inhibinput.h"
#include "constants.h"
#include "helpers.h"
#include "diagramscene.h"

#include <QPainter>

InhibInput::InhibInput() : InputSlot(INHIBITION_INPUT_NAME), m_line(this)
{
	m_line.setLine(0, -5, 0, -30);
}

void InhibInput::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	InputSlot::paint(painter, option, widget);

	QGraphicsScene *scene_ = scene();
	DiagramScene *dscene = dynamic_cast<DiagramScene *>(scene_);

	if (dscene == NULL) {
		informUserAndCrash("Could not cast the scene into a DiagramScene!");
	}

	QPointF p = m_line.line().p2();
	qreal rx, ry;
	rx = ry = 2;
	painter->drawEllipse(p, rx, ry);
	QPainterPath path;
	path.addEllipse(p, rx, ry);
	painter->fillPath(path, Qt::black);

	// If there are links connected to inhib, show it
	if (!m_inputs.empty())
		setOpacity(1.0);
	else if (dscene->leftBtnDown() && dscene->line() != nullptr && m_dist < DISTANCE_SHOW_INHIBITION) {
		// Display the inhibition link only when drawing a line, and near enough
		/*
		 * Create a nice exponential progression on opacity when the user approaches its mouse
		 * cursor.
		 * See https://www.desmos.com/calculator/k37l2jdcex for the graph
		 */
		setOpacity(exp(-m_dist/90.0));
	} else
		setOpacity(0.1);
}
