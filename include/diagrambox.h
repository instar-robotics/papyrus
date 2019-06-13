/*
  Copyright (C) INSTAR Robotics

  Author: Nicolas SCHOEMAEKER

  This file is part of papyrus <https://github.com/instar-robotics/papyrus>.

  papyrus is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  papyrus is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DIAGRAMBOX_H
#define DIAGRAMBOX_H

#include "types.h"
#include "slot.h"
#include "outputslot.h"
#include "inputslot.h"
#include "rossession.h"
#include "inhibinput.h"

#include <set>

#include <QGraphicsItem>
#include <QIcon>
#include <QUuid>
#include <QGraphicsSvgItem>
#include <QGraphicsSceneMouseEvent>

class Script;
class ActivityVisualizer;
class OpenGLProxy;
class ShaderProxy;

/**
 * @brief The DiagramBox class is the main class that represents a "box" or "neural function".
 * This has a name, an icon, a type (scalar or matrix), etc.
 * It can have @InputSlot s and @OutputSlot s attached.
 */

class DiagramBox : public QObject, public QGraphicsItem
{
	Q_OBJECT

public:
	static int getType();


	// TODO: implement a copy constructor that should change the uuid and remove the connected links
	explicit DiagramBox(const QString &name,
	                    OutputSlot *outputSlot,
	                    std::vector<InputSlot *> inputSlots,
	                    const QString &description = QString(),
	                    const QUuid &uuid = 0,
	                    InhibInput *inhibInput = nullptr,
	                    QGraphicsItem *parent = 0);
	~DiagramBox();

	QRectF boundingRect() const override;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	QVariant itemChange(GraphicsItemChange change, const QVariant &value);

	Script *getScript();
	bool checkIfBoxInvalid();
	void updateTooltip();

	void setOutputSlotPos();

	void updateSizeIcon();
	virtual void createFunctionIcon();

	QString scriptName();

	QString name() const;
	void setName(const QString &name);

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	int type();

	QString descriptionFile() const;
	void setDescriptionFile(const QString &descriptionPath);

	QString description() const;
	void setDescription(const QString &description);

	OutputSlot *outputSlot() const;
	void setOutputSlot(OutputSlot *outputSlot);

	std::vector<InputSlot *> inputSlots() const;
	void setInputSlots(const std::vector<InputSlot *> &inputSlots);

	OutputType outputType() const;
	void setOutputType(const OutputType &outputType);

	int rows() const;
	void setRows(int rows);

	int cols() const;
	void setCols(int cols);

	bool saveActivity() const;
	void setSaveActivity(bool saveActivity);

	qreal bWidth() const;

	qreal bHeight() const;

	qreal tHeight() const;

	QGraphicsSvgItem &sizeIcon();

	bool publish() const;
	void setPublish(bool publish);

	QString topic() const;
	void setTopic(const QString &topic);

	QString iconFilepath() const;
	void setIconFilepath(const QString &value);

	QString libname() const;
	void setLibname(const QString &libname);

	QString title() const;
	void setTitle(const QString &title);

	MatrixShape matrixShape() const;
	void setMatrixShape(const MatrixShape &matrixShape);

	bool isInvalid() const;
	void setIsInvalid(bool isInvalid);

	BoxInvalidReason invalidReason() const;
	void setInvalidReason(const BoxInvalidReason &invalidReason);

	bool swapCandidate() const;
	void setSwapCandidate(bool swapCandidate);

	bool isActivityVisuEnabled() const;
	void setIsActivityVisuEnabled(bool IsActivityVisuEnabled);

	ActivityVisualizer *activityVisualizer() const;
	void setActivityVisualizer(ActivityVisualizer *activityVisualizer);

	InhibInput *inhibInput() const;
	void setInhibInput(InhibInput *inhibInput);

	void setDisplayedProxy(ShaderProxy *displayedProxy);

	ShaderProxy *getDisplayedProxy() const;

	int getRows() const;

	int getCols() const;

	bool isCommented() const;

	void setIsCommented(bool isCommented);
protected:

	QString m_name;    // Name of the function
	QString m_title;   // Title of the box (user-friendly & customizable name)

	qreal m_bWidth;  // Overall width of the function's box
	qreal m_bHeight; // Overall height of the function's box
	qreal m_tHeight; // Height of the space in which th function's name is written

	MatrixShape m_matrixShape; // Shape (vector, row vector or col vector) if matrix

	QGraphicsSvgItem m_sizeIcon;     // Contains the svg that hints the box's size
	QGraphicsSvgItem *m_functionIcon; // Contains the SVG icon of the function

private:
	QUuid m_uuid;          // Unique ID of the function's box (to identify links for instance)
	QString m_libname;     // Name of the library this function belongs to (for kheops's linking)

	QString m_description; // Description of the function
	QString m_descriptionFile; // Path to its XML description file (to get the icon when saving)

	OutputSlot *m_outputSlot;  // The output slot for this function's box
	std::vector<InputSlot *> m_inputSlots; // The set of input slots for this function's box
	InhibInput *m_inhibInput;              // Special inhibition input, common to all boxes

	int m_rows;              // Number of rows in the output (if matrix)
	int m_cols;              // Number of columns in the output (if matrix)

	/*
	 * Whether or not this function saves its activity in memory. It defaults to false. Saving the
	 * activity is not free and should only be done for activities that you need to recover if
	 * the script crashes and the robot cannot rebuild from its environment.
	 * For instance the path integration, or a counter, or smth.
	 * This should normally only be scalar values or small matrices
	 */
	bool m_saveActivity;
	bool m_publish;           // whether to publish this function's output on ROS
	QString m_topic;          // name of the topic in which to publish the output

	QString m_iconFilepath;         // Filepath of the icon used (needed to create the svg)

	bool m_IsActivityVisuEnabled;  // Wether or not we are visualizing this box's activity
//	ActivityFetcher *m_activityFetcher; // The thread that subscribes to ROS topics
//	DiagramChart *m_activityChart; // A Chart to display this box's activity

	bool m_isInvalid; // Whether this box is invalid
	BoxInvalidReason m_invalidReason; // Why this box is invalid
	bool m_swapCandidate; // Set to true when the user is dropping another box on top of this one

	QPointF m_oldPos; // Start position when moved (to enable undo)

	ActivityVisualizer *m_activityVisualizer;
	ShaderProxy *m_displayedProxy;
	bool m_isCommented;  // Whether this Box is commented or not (for the execution)

private slots:
	void deleteOpenGLDisplay();

signals:
	void boxSelected(DiagramBox *); // Fired when the box is clicked on (used to signal PropertiesPanel)
	void boxDestroyed(); // emited when the destructor is called, used by the ActivityVisualizer to delete itself
};

#endif // DIAGRAMBOX_H
