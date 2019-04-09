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

#ifndef ACTIVITYFETCHER_H
#define ACTIVITYFETCHER_H

#include "diagrambox.h"

#include <QThread>

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

/**
 * @brief The ActivityFetcher class is a thread that subscribes to a box's topic in order to
 * fetch its activity output, and then push the values to a visualizer.
 */

class ActivityFetcher : public QThread
{
	Q_OBJECT
public:
	explicit ActivityFetcher(const QString &topicName, DiagramBox *box, QObject *parent = nullptr);
	~ActivityFetcher();

	void run() override;

private:
	void fetchScalar(const std_msgs::Float64::ConstPtr& scalar);
	void fetchMatrix(const std_msgs::Float64MultiArray::ConstPtr& mat);

	QString m_topicName;
	DiagramBox *m_box;
	bool m_shouldQuit;

signals:
	void newScalar(qreal scalar);
	void newMatrix(const QList<qreal> matrix&);
};

#endif // ACTIVITYFETCHER_H
#ifndef ADDBOXCOMMAND_H
#define ADDBOXCOMMAND_H

#include "diagrambox.h"
#include "diagramscene.h"

#include <QUndoCommand>

/**
 * @brief The AddBoxCommand class represents adding a @DiagramBox to aa @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddBoxCommand : public QUndoCommand
{
public:
	AddBoxCommand(DiagramScene *scene, DiagramBox *box, const QPointF &initialPos, QUndoCommand *parent = nullptr);
	~AddBoxCommand();

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene; // The scene in which the box should be added
	DiagramBox *m_box;     // The DiagramBox to add to the scene
	QPointF m_initialPos;  // The initial position at which the box is added
};

#endif // ADDBOXCOMMAND_H
#ifndef ADDLINKCOMMAND_H
#define ADDLINKCOMMAND_H

#include "diagramscene.h"
#include "link.h"
#include "inputslot.h"
#include "outputslot.h"

#include <QUndoCommand>

/**
 * @brief The AddLinkCommand class represents adding a @Link to a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddLinkCommand : public QUndoCommand
{
public:
	AddLinkCommand(DiagramScene *scene, Link *link, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene; // The scene in which the link is added
	Link *m_link;          // The link to add to the scene
	OutputSlot *m_from;    // The output slot the link is from
	InputSlot *m_to;       // The input slot the link goes to
	bool m_isFirst;        // Flag that indicates wheather a redo() action is the first one
};

#endif // ADDLINKCOMMAND_H
#ifndef ADDZONECOMMAND_H
#define ADDZONECOMMAND_H

#include "diagramscene.h"
#include "zone.h"

#include <QUndoCommand>

/**
 * @brief The AddZoneCommand class represents adding a @Zone to a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddZoneCommand : public QUndoCommand
{
public:
	AddZoneCommand(DiagramScene *scene, Zone *zone, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene;  // The scene in which to add the zone
	Zone *m_zone;           // The zone to add to the scene
};

#endif // ADDZONECOMMAND_H
#ifndef CATEGORY_H
#define CATEGORY_H

#include "function.h"

#include <vector>
#include <QTreeWidgetItem>

/**
 * @brief The Category class holds a number of neural @Function 's descriptions that are part of the
 * same category.
 * This is used with the @Library and the @LibraryPanel. The idea is to group similar functions
 * by themes. The categories are simply made by parsing the directory name in which the function's
 * descriptions are saved.
 */

class Category : public QTreeWidgetItem
{
public:
    Category(const QString &name);

    QString name() const;
    void setName(const QString &name);

private:
    QString m_name;
};

#endif // CATEGORY_H
#ifndef CHANGELOG_H
#define CHANGELOG_H

#include <QString>

QString changelog = "<h3>CHANGELOG</h3>"
                    "<ul>"

                    "<li><strong>v0.5.1</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>When saving the same script, without adding any boxes, and just making "
                    "position changes, or changing the weight of a link, the order in which "
                    "functions and links were written in the XML was not deterministic. It has been "
                    "fixed, and now both functions and links are written in order of their UUID. "
                    "Moving boxes or changing link weights will produce only a small git diff for "
                    "the coordinates / weights.</li>"
                    "<li>"
                    "<li>There was a segfault when closing Papyrus after opening two scripts. This "
                    "prevented the normal cleaning procedures to happen and thus settings to be "
                    "saved (among other things). This is now fixed.</li>"
                    "<li>The libraries in the library panel on the left are not sorted "
                    "alphabetically (they were sorted in the reverse order)</li>"
                    "<li>Lots of XML information about the function boxes were saved in the script's"
                    " XML and thus any changed in alexandria (such as the icon, the name of inputs, "
                    "etc.) were not being reloaded. Now only the minimum information is stored in "
                    "the XML, instead, much of the information regarding a box are being re-parsed "
                    "from the library when a script is opened.</li>"
                    "<li>There was a compilation error on older Qt version, because a QStringRef "
                    "could not be used in the same manner, this was converted back to QString to fix"
                    " this.</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.5.0</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Parse <description> tags from XML description files and display a "
                    "function's description (and its input) when hovering the library</li>"
                    "<li>Fix the graphical glitch where a link seemed to \"go out in the distance\" "
                    "when a Comment Zone was resized and the box was now out of the zone</li>"
                    "<li>The full version MAJOR.MINOR.BUGFIX is now displayed in the Home Page and "
                    "the About dialog (the bugfix version was missing) which allows the user to know"
                    " which version he is running</li>"
                    "<li>Only one Comment Zone was parsed & loaded when a script was opened, this is"
                    " now fixed</li>"
                    "<li>Function boxes used to be transparent which made them weird-looking when "
                    "placed inside Comment Zones, now their background is white</li>"
                    "<li><strong>(new feature)</strong> Papyrus now periodically checks the gitlab repository for a "
                    "new release and when it finds one, it warns the user with a dialog</li>"
                    "<li><strong>(new feature)</strong> Papyrus will now re-open with the last "
                    "opened scripts (and the last active script) by default. It is possible, however"
                    " to disable this feature in the options</li>"
                    "<li><strong>(new feature)</strong> The full CHANGELOG is now visible in the "
                    "Help > CHANGELOG menu. Also, when it's the first time a new version is "
                    "launched, the CHANGELOG is automatically displayed.</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.4.2</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Fix the big issue when events on properties panel (such as clicking OK, CANCEL, "
                    "or pressing ENTER) was propagated to all open scenes. It was thus unsafe to edit "
                    "several scripts at the same time. Now it should be safe.</li>"
                    "<li>Auto scaling features for the bar and graph visualization was not symmetric, so "
                    "it was difficult to read where the 0 lien was. Now it is symmetric, the 0 line is "
                    "thus always centered</li>"
                    "<li>When visualizing several neurons, the bars colors were cycled, and when no "
                    "more colors were found, it would switch to alpha channel, which made it "
                    "difficult to read. Now, all neurons have the same color</li>"
                    "<li>Papyrus segfaulted when a new script was created or opened and the ROS "
                    "master was down. This is now fixed.</li>"
                    "<li>It was not possible to save the scripts which were invalid. This was "
                    "problematic when you had to save your work and go, and did not have the time to"
                    " fix the problem. Now there's still a warning message, but saving is still"
                    " performed.</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.4.1</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Papyrus used to segfault when naming or renaming a script with a space in "
                    "the name, this was due to ROS not accepting spaces in topic names</li>"
                    "<li>The multiple XML argument for inputs is now optional and will default to "
                    "false when not specified. It is now encouraged not to add multiple = \"false\" "
                    "to make XML files more lightweight.</li>"
                    "<li>Papyrus did not check if an input was multiple or not before creating Link."
                    " Now it does, so it's not possible to connect several functions to an input "
                    "that is marked multiple</li>"
                    "<li>There was a problem saving the secondary argument: it used to be that Links"
                    " were saved as secondary if and only if it was a link from the same box it "
                    "started. Now it checks the real value.</li>"
                    "<li>When saving a script for the first time, it will use the script's name as "
                    "the default value for the XML file. So that the user doesn't have to manually "
                    "type it. Note: the script name is sanitized, so it means it is safe to use "
                    "spaces in script name: Visual Docking will be transformed as VisualDocking.xml."
                    " So please use spaces if you want to.</li>"
                    "<li>When opening a script, the state of the interface buttons (play, pause, "
                    "stop) was not initiated, thus was actually random. Now they are initiated to "
                    "the state where the script had not been launched (thus \"play\" visible only).</li>"
                    "<li>When a script was saved, there one warning message per constant function "
                    "box talking about missing description file. This is now fixed (built-in "
                    "Constants do not have description files)</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.4.0</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Implement a reliable, asynchronous communication with Kheops nodes.<br>"
                    "Now, starting, pausing and stopping a node is reliable (either when creating "
                    "and launching a node, or connecting to an existing node). The interface is "
                    "reliable and updated to the node's reported status.<br>"
                    "Basically, if you see a play button but no stop button, it means the script is "
                    "not launched. Is you see a play and a stop button, it means the script is "
                    "launched, but paused (clicking \"play\" will resume the execution).<br>"
                    "If you see a pause button, it means the script is launched and running "
                    "(clicking the pause button will pause the script's execution).>/li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.3.0</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Autosave feature is now implemented: when working on a script, it is "
                    "autosaved every 60 seconds (in a .autosave file). When one tries to open a "
                    "script, Papyrus will detect if there is an autosaved version of this file and "
                    "offer to open it instead (one can accept or refuse). Autosaved files are "
                    "destroyed when the script file is saved</li>"
                    "<li>When a script was opened, there was a bug where the script just before it "
                    "in the tab list would take its name. Thus we would have two tabs with the same "
                    "name. This is now fixed</li>"
                    "<li>The input and output slots of functions are now colorized according to "
                    "their types. At the moment, STRING-based slots are colorized cyan, SCALAR-based"
                    " types are left white and MATRIX-based slots are colorized pink. This makes it "
                    "much easier to see the compatible inputs</li>"
                    "<li>Function boxes can now have individual names. By default they don't have a "
                    "name and they function name is displayed. To give a name, edit the field in the"
                    " properties panel. When a function box has a name, it is displayed instead of "
                    "its function name. To remove a name (and restore displaying the function's "
                    "name), simply delete the name and validate.</li>"
                    "<li>Modifications in the properties panel can now be validated by pressing Enter "
                    "and cancelled by pressing ESCAPE (before this, one had to click on the Ok or "
                    "Cancel buttons ; now both choices are available).</li>"
                    "<li>Changes in the properties panel are now reflected on the scene as soon as "
                    "Enter is pressed (or Ok is clicked). Previous to this, it was necessary to "
                    "click on the scene with the mouse to trigger a repaint. This is no longer "
                    "necessary.</li>"
                    "<li>When changing some parameters in the properties panel, the script is now "
                    "set as \"modified\".</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.2.2</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Fix the icons for the ConstantBox that were going outside the box (this bug"
                    " happens when boxes were resized from 3 squares to 2). Now their icons are "
                    "displayed properly</li>"
                    "<li>I detected that if there was an issue (such as a segfault) during the save "
                    "operation, not only would the save operation fail, but the script file would be"
                    " emptied!<br>"
                    "This means it was possible to potentially lose a lot a work by trying to save.<br>"
                    "Now this is fixed: the saving is performed in a temporary file, and only when "
                    "this temporary file is written and successfully closed, it is copied on top of "
                    "the original script file.<br>"
                    "In case there's a segfault or some other error, only the modifications are lost"
                    " but the original script file is left untouched.</li>"
                    "</ul>"
                    "</li>"


                    "</ul>";

#endif // CHANGELOG_H
#ifndef CONNECTIVITYWINDOW_H
#define CONNECTIVITYWINDOW_H

#include <QTabWidget>

namespace Ui {
class ConnectivityWindow;
}

class ConnectivityWindow : public QTabWidget
{
    Q_OBJECT

public:
    explicit ConnectivityWindow(QSize inputSize, QSize outputSize, QWidget *parent = 0);
    ~ConnectivityWindow();

private:
    Ui::ConnectivityWindow *ui;
    QSize m_inputSize;   // Size of the input matrix
    QSize m_outputSize;  // Size of the output matrix
};

#endif // CONNECTIVITYWINDOW_H
#ifndef CONSTANTDIAGRAMBOX_H
#define CONSTANTDIAGRAMBOX_H

#include "diagrambox.h"
#include "outputslot.h"
#include "inputslot.h"

#include <QGraphicsItem>
#include <QIcon>
#include <QUuid>
#include <QPainter>

class ConstantDiagramBox : public DiagramBox
{
	Q_OBJECT

public:
	explicit ConstantDiagramBox(const QString &name,
	                            const QIcon &icon,
	                            OutputSlot *outputSlot,
	                            const QUuid &uuid = 0,
	                            QGraphicsItem *parent = 0);

	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};

#endif // CONSTANTDIAGRAMBOX_H
#ifndef CONSTANTFUNCTION_H
#define CONSTANTFUNCTION_H

#include "function.h"
#include "types.h"

class ConstantFunction : public Function
{
public:
    ConstantFunction(const QString &name,
                     const QString &iconPath,
                     const QIcon &icon,
                     const OutputType outputType);
};

#endif // CONSTANTFUNCTION_H
#ifndef CONSTANTS
#define CONSTANTS

/**
 * A set of project-wide constants
 */

// Define the name of the application
#define APP_NAME "Papyrus"

// Define the scroll factor for the QGraphicsView
#define SCALE_FACTOR 1.2

// Define the prefix in which to search for ressources
// TEMPORARY: should be replaced by a global 'settings' file for Papyrus
#define RESOURCE_DIR "/home/nschoe/workspace/Qt/papyrus/usr/share/"
// #define RESOURCE_DIR "/usr/share/"

// Define the size of the icons in the left 'Library' pane (in px)
#define LIBRARY_ICON_SIZE 40

// Define the default name for a new script
#define NEW_SCRIPT_DEFAULT_NAME "Untitled"

// Define the margin added to the scene's rectangle when resizing it (in px)
#define SCENE_RECT_MARGIN 200

// Define the name of the root element XML tag in the function's description files
#define XML_ROOT_ELEM "description"

// Major version number
#define MAJOR_VERSION 0

// Minor version number
#define MINOR_VERSION 5

// Bugfix version number
#define BUGFIX_VERSION 1

// Define the maximum number of rows / columns a matrix can have (for the spinbox)
#define MAX_ROWS 100000
#define MAX_COLS 100000

// Define the minimum and maximum allowed weight (for the double spinbox)
#define MIN_WEIGHT -10000000
#define MAX_WEIGHT 10000000

// Define the number of decimals for the double spinbox for link's weight
#define LINKS_NB_DECIMALS 10

// Define the minimum and maximum time value (for the double spinbox)
#define MIN_TIME_VALUE 0 //0.001
#define MAX_TIME_VALUE 10000

// Define the z-value for the links (used to put them behind slots to prevent hiding the slots)
#define LINKS_Z_VALUE -1.0

// Define the z-value for the data visualization windows (above boxes and links)
#define DATA_Z_VALUE 10

// Define the z-value for the rectangular comments (under everything)
#define COMMENTS_Z_VALUE -10.0

// Define the z-value for the neural boxes
#define BOXES_Z_VALUE 5

// Define the time (in minutes) after which the user is notified about modified, unsaved scripts
#define TIME_WARN_MODIFIED 10

// Define the organisation name, and domain for use with QSettings
#define ORGA "INSTAR Robotics"
#define DOMAIN "instar-robotics.com"

// Define the period for the autosave script
#define AUTOSAVE_PERIOD 60000 // 1 minute

// Time (in ms) during which status bar messages are displayed
#define MSG_DURATION 3000

#endif // CONSTANTS

#ifndef DATAFETCHER_H
#define DATAFETCHER_H

#include "types.h"

#include <QThread>
#include <QString>
#include <QList>
#include <ros/ros.h>

//QT_CHARTS_USE_NAMESPACE

class DataFetcher : public QThread
{
	Q_OBJECT

public:
	explicit DataFetcher(const QString &topicName, QObject *parent = nullptr);

	bool shouldQuit() const;
	void setShouldQuit(bool shouldQuit);

	VisualizationType visType() const;
	virtual void setVisType(VisualizationType type) = 0;

protected:
	QString m_topicName;
	bool m_shouldQuit;
	VisualizationType m_visType;

signals:
	void newMatrix(QList<double> *values);
};

#endif // DATAFETCHER_H
#ifndef DATAVISUALIZATION_H
#define DATAVISUALIZATION_H

#include "datafetcher.h"
#include "rossession.h"

#include <QWidget>
#include <QMenuBar>
#include <QMenu>
#include <QGraphicsScene>
#include <QThread>
#include <QString>

//QT_CHARTS_USE_NAMESPACE

class DiagramBox;
//class QBarSet;

class DataVisualization : public QWidget
{
	Q_OBJECT

public:
	DataVisualization(QWidget *parent = nullptr,
	                  ROSSession *rosSession = nullptr,
	                  QGraphicsScene *scene = nullptr,
	                  DiagramBox *box = nullptr);
	~DataVisualization();

protected:
	ROSSession *m_rosSession;
	QGraphicsScene *m_scene;
	DiagramBox *m_box;
	QMenuBar *m_menuBar;
	QMenu *m_typeMenu;
	DataFetcher *m_dataFetcher;
};

#endif // DATAVISUALIZATION_H
#ifndef DIAGRAMBOX_H
#define DIAGRAMBOX_H

#include "types.h"
#include "slot.h"
#include "outputslot.h"
#include "inputslot.h"
#include "datavisualization.h"
#include "rossession.h"

#include <set>

#include <QGraphicsItem>
#include <QIcon>
#include <QUuid>
#include <QGraphicsSvgItem>
#include <QGraphicsProxyWidget>

class Script;

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
	                    const QIcon &icon,
	                    OutputSlot *outputSlot,
	                    std::vector<InputSlot *> inputSlots,
	                    const QUuid &uuid = 0,
	                    QGraphicsItem *parent = 0);
	~DiagramBox();

	QRectF boundingRect() const override;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	Script *getScript();
	bool checkIfBoxInvalid();
	void updateTooltip();

	void showDataVis(ROSSession *rosSession);
	void setOutputSlotPos();

	QString scriptName();

	QString name() const;
	void setName(const QString &name);

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	int type();

	QString descriptionFile() const;
	void setDescriptionFile(const QString &descriptionPath);

	QIcon icon() const;
	void setIcon(const QIcon &icon);

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

	QGraphicsSvgItem *sizeIcon() const;
	void setSizeIcon(QGraphicsSvgItem *sizeIcon);

	bool publish() const;
	void setPublish(bool publish);

	QString topic() const;
	void setTopic(const QString &topic);

	QString iconFilepath() const;
	void setIconFilepath(const QString &value);

	QString libname() const;
	void setLibname(const QString &libname);

	DataVisualization *dataVis() const;
	void setDataVis(DataVisualization *dataVis);

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

protected:
	QVariant itemChange(GraphicsItemChange change, const QVariant &value);

	QString m_name;    // Name of the function
	QString m_title;   // Title of the box (user-friendly & customizable name)

	qreal m_bWidth;  // Overall width of the function's box
	qreal m_bHeight; // Overall height of the function's box
	qreal m_tHeight; // Height of the space in which th function's name is written

	MatrixShape m_matrixShape; // Shape (vector, row vector or col vector) if matrix

private:
	QUuid m_uuid;      // Unique ID of the function's box (to identify links for instance)
	QString m_libname; // Name of the library this function belongs to (for kheops's linking)
	QIcon m_icon;      // Icon representing the function

	QString m_descriptionFile; // Path to its XML description file (to get the icon when saving)

	OutputSlot *m_outputSlot;  // The output slot for this function's box
	std::vector<InputSlot *> m_inputSlots; // The set of input slots for this function's box

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
	QGraphicsSvgItem *m_sizeIcon; // Contains the svg that hints the box's size

	DataVisualization *m_dataVis; // The data visualization window for this box
	QGraphicsProxyWidget *m_dataProxy; // The proxy for the data visualization window

	bool m_isInvalid; // Whether this box is invalid
	BoxInvalidReason m_invalidReason; // Why this box is invalid
	bool m_swapCandidate; // Set to true when the user is dropping another box on top of this one

	QPointF m_oldPos; // Start position when moved (to enable undo)

private slots:
	void onDataVisClosed();
signals:
	void boxSelected(DiagramBox *); // Fired when the box is clicked on (used to signal PropertiesPanel)
};

#endif // DIAGRAMBOX_H
#ifndef DIAGRAMCHART_H
#define DIAGRAMCHART_H

#include "diagrambox.h"
#include "types.h"

#include <QChart>
#include <QBarSeries>
#include <QHorizontalBarSeries>
#include <QBarSet>
#include <QValueAxis>
#include <QGraphicsSceneHoverEvent>

QT_CHARTS_USE_NAMESPACE

class DiagramChart : public QChart
{
	Q_OBJECT

public:
	explicit DiagramChart(DiagramBox *box,
	                      QGraphicsItem *parent = nullptr,
	                      Qt::WindowFlags wFlags = Qt::WindowFlags());

	void hoverMoveEvent(QGraphicsSceneHoverEvent *evt);
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);

private:
	DiagramBox *m_box;          // The associated DiagramBox
	int m_size;                 // The number of data points in this chart
	MatrixShape m_matrixShape;  // Used to know if we display horizontally or vertically

	QBarSet m_barSet;

	QBarSeries m_barSeries;
	QValueAxis m_yAxis;

	QHorizontalBarSeries m_horizontalBarSeries;
	QValueAxis m_xAxis;

	qreal m_barMin;
	qreal m_barMax;

	qreal m_width;
	qreal m_height;

	ResizeType m_resizeType; // Type of resizing operation we are currently performing

public slots:
	void updateBarValues(const QList<qreal> matrix&);
};

#endif // DIAGRAMCHART_H
#ifndef DIAGRAMSCENE_H
#define DIAGRAMSCENE_H

#include "script.h"
#include "outputslot.h"
#include "inputslot.h"
#include "diagrambox.h"
#include "zone.h"

#include <QGraphicsScene>
#include <QUuid>
#include <QUndoStack>

// Forward declaration because of recursive include
class Script;
class PapyrusWindow;

/**
 * @brief The DiagramScene class represents the canvas on which neural scripts are drawn.
 * This is where @DiagramBox es are drag-n-dropped. There is one DiagramScene per script.
 */
class DiagramScene : public QGraphicsScene
{
	Q_OBJECT

public:
	explicit DiagramScene(QObject *parent = 0);
	~DiagramScene();

	void addBox(DiagramBox *newBox, const QPointF &position);

	bool checkForInvalidLinks();
	bool checkForInvalidity();

	void updateSceneRect();

	bool shouldDrawGrid() const;
	void setShouldDrawGrid(bool shouldDrawGrid);

	void removeItem(QGraphicsItem *item);

	int gridSize() const;

	Script *script() const;
	void setScript(Script *script);

	bool leftBtnDown() const;

	QGraphicsLineItem *line() const;

	PapyrusWindow *mainWindow() const;

	bool displayLabels() const;
	void setDisplayLabels(bool displayLabels);

	bool rightBtnDown() const;
	void setRightBtnDown(bool rightBtnDown);

	QGraphicsRectItem *rect() const;
	void setRect(QGraphicsRectItem *rect);

	QUndoStack *undoStack() const;
	void setUndoStack(QUndoStack *undoStack);

public slots:
	void toggleDisplayGrid(bool shouldDraw);
	void onOkBtnClicked(bool);
	void onCancelBtnClicked(bool);
	void onDisplayVisuClicked(bool);

protected:
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *evt);
	void dragEnterEvent(QGraphicsSceneDragDropEvent *evt);
	void dragLeaveEvent(QGraphicsSceneDragDropEvent *evt);
	void dragMoveEvent(QGraphicsSceneDragDropEvent *evt);
	void dropEvent(QGraphicsSceneDragDropEvent *evt);
	void keyPressEvent(QKeyEvent *evt);
	void deleteItem(Link *link);
	void deleteItem(DiagramBox *box);
	void deleteItem(Zone *zone);

	void drawBackground(QPainter *painter, const QRectF &rect);

private:
	PapyrusWindow *m_mainWindow; // A pointer to the main Papyrus window widget
	bool m_leftBtnDown;
	bool middleBtnIsDown;
	bool m_rightBtnDown;
	bool m_shouldDrawGrid;   // Whether to draw the grid or not
	int m_gridSize;          // Size (in px) of the grid
	QGraphicsLineItem *m_line; // The current line being drawn while clicking
	QGraphicsRectItem *m_rect; // The current rectangular section being drawn while clicking
	OutputSlot *m_oSlot;     // The slot from which the line being drawn originates
	Script *m_script;        // The script to which this scene is associated
	bool m_displayLabels;    // Whether or not to display input slots's names
	bool m_prevDisplayLabels;// Remembers the value of 'displayLabel' when creating links (to restore afterward)
	QUndoStack *m_undoStack; // The stack to allow for undo / redo commands

private slots:
	void onSelectionChanged();

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);

};

#endif // DIAGRAMSCENE_H
#ifndef DIAGRAMVIEW_H
#define DIAGRAMVIEW_H

#include <QGraphicsView>

/**
 * @brief The DiagramView class is used to render a @DiagramScene (the @DiagramScene holds
 * the data: the items added, their positions, etc.) and the DiagramView displays it. This
 * separation by Qt allows several views to be attached to a single scene (we will use
 * something like this when we implement the minimap).
 */
class DiagramView : public QGraphicsView
{
	Q_OBJECT
public:
	explicit DiagramView(QWidget *parent = 0);
	DiagramView(QGraphicsScene *scene, QWidget *parent = 0);

signals:

protected:
	void wheelEvent(QWheelEvent *evt);

public slots:
};

#endif // DIAGRAMVIEW_H
#ifndef FUNCTION_H
#define FUNCTION_H

#include "outputslot.h"
#include "inputslot.h"

#include <vector>

#include <QString>
#include <QIcon>
#include <QTreeWidgetItem>

/**
 * @brief The Function class describes a neural function for the @Library and @LibraryPanel
 * and comes from parsing a valid XML description file. Basically, the @Category class
 * holds several @Function.
 * The @Function class is only meant to be stored inside the @Library, but the actual neural
 * function is created as a @DiagramBox from this lightweight @Function object when it is
 * dropped on the @DiagramScene.
 */

class Function : public QTreeWidgetItem
{
public:
	Function(const QString &path);

	void updateTooltip();

	QString name() const;
	void setName(const QString &name);

	std::vector<InputSlot *> inputs() const;

	void addInputSlot (InputSlot *slot);

	OutputSlot *output() const;
	void setOutput(OutputSlot *output);

	QString descriptionFile() const;

	bool constant() const;
	void setConstant(bool constant);

	QString libName() const;
	void setLibName(const QString &libName);

	QString iconFilepath() const;
	void setIconFilepath(const QString &value);

	QString description() const;
	void setDescription(const QString &description);

	MatrixShape matrixShape() const;
	void setMatrixShape(const MatrixShape &matrixShape);

protected:
	QString m_name;
	QString m_descriptionFile;
	//    QIcon m_icon;
	std::vector<InputSlot *> m_inputs;
	OutputSlot *m_output;
	bool m_constant;   // Indicate whether this represents a constant input
	QString m_libName; // The name of the lib it belongs to, used for kheops to know where to look
	QString m_iconFilepath; // Filepath for the SVG icon, used when dropping the box on scene
	QString m_description;  // Description of the function
	MatrixShape m_matrixShape;
};

#endif // FUNCTION_H
#ifndef HELPERS_H
#define HELPERS_H

#include "types.h"
#include "papyruswindow.h"
#include "slot.h"

#include <ros/ros.h>

#include <QApplication>
#include <QString>
#include <QGraphicsSvgItem>
#include <QList>
#include <QDebug>
#include <QColor>

/**
 * This file provides a set of helpers that are used throughout this project.
 */

PapyrusWindow *getMainWindow();

QString outputTypeToString(const OutputType &outputType);
OutputType stringToOutputType(const QString &str);

QString inputTypeToString(const InputType &inputType);
InputType stringToInputType(const QString &str);

QString timeUnitToString(const TimeUnit &unit);

bool canLink(const OutputType &from, const InputType &to);

bool shapesMatch(DiagramBox *from, InputSlot *to, LinkInvalidReason *reason = nullptr);

void informUserAndCrash(const QString &text, const QString &title = QObject::tr("Papyrus is about to crash!"));

void rescaleSvgItem(QGraphicsSvgItem *svg, const QSizeF &size, const QPointF &pos, bool center = true);

void updateSizeIcon(DiagramBox *box);

bool areLinked(OutputSlot *oSlot, InputSlot *iSlot);

bool isFull(InputSlot *iSlot);

bool fileExists(const std::string& filename);

QList<QString> getKheopsNodes();

QString snakeCaseToPretty(const QString &str);

QString connectivityToString(const Connectivity &conn);

Connectivity stringToConnectivity(const QString &str);

QString mkTopicName(const QString &scriptName, const QString &topicName);

QString sanitizeTopicName(const QString &name);

bool isTopicNameValid(const QString &name);

QString ensureSlashPrefix(const QString &name);

QColor getTypeColor(const InputType inputType);

QColor getTypeColor(const OutputType outputType);

QString mkFilenameFromScript(const QString &scriptName);

qreal qrealAbsMax(const qreal a, qreal b);

MatrixShape stringToMatrixShape(const QString &str);

QString matrixShapeToString(const MatrixShape shape);

#endif // HELPERS_H
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
#ifndef INPUTSLOT_H
#define INPUTSLOT_H

#include "slot.h"
#include "types.h"

#include <QString>
#include <set>
#include <QGraphicsSimpleTextItem>
#include <QUuid>

class Link;

/**
 * @brief The InputSlot class is attached to a @DiagramBox. This is represented as the input
 * little circle to a @DiagramBox. Its job is to keep track of all @Link s from other
 * @Diagrambox es.
 * There is one @InputSlot per <input> in the function's XML description file.
 */
class InputSlot : public Slot
{
	Q_OBJECT

public:
	explicit InputSlot();
	explicit InputSlot(const QString &name);
	~InputSlot();

	bool multiple() const;
	void setMultiple(bool allowMultiple);

	std::vector<Link *> inputs() const;

	void addInput(Link *input, bool ignoreFull = false);
	void removeInput(Link *input);

	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
	QRectF boundingRect() const override;

	void updateLinks();

	InputType inputType() const;
	void setInputType(const InputType &inputType);

	bool canLink() const;
	void setCanLink(bool canLink);

	QGraphicsSimpleTextItem *label() const;
	void setLabel(QGraphicsSimpleTextItem *label);

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	bool checkSize() const;
	void setCheckSize(bool checkSize);

	QString description() const;
	void setDescription(const QString &description);

	MatrixShape matrixShape() const;
	void setMatrixShape(const MatrixShape &matrixShape);

private:
	QUuid m_uuid;              // Unid ID for the slot (used for kheops)
	bool m_multiple;           // Whether this slot can receive several links
	std::vector<Link *> m_inputs; // The set of links connected to this slot
	InputType m_inputType;     // Indicate type and connectivity of this input
	bool m_canLink;            // Indicate if this input can be linked to the current output when creating a Link
	QGraphicsSimpleTextItem *m_label; // A label that contains this input's name
	bool m_checkSize;          // Whether or not this input must check that sizes are correct (on SCALAR_MATRIX)
	QString m_description;     // Description of the input
	MatrixShape m_matrixShape; // The shape of the matrix it can accept
signals:
	void slotFull(); // Fired when trying to add a second input to a slot that doesn't allow multiple
};

#endif // INPUTSLOT_H
#ifndef LIBRARY_H
#define LIBRARY_H

#include "category.h"

#include <vector>

/**
 * @brief The Library class holds the @Categorie s which in turn hold all the @Function s
 * that have a valid XML description file. These @Function are then used as building blocks for
 * creating neural programs (by converting them to @DiagramBox es).
 */

class Library
{
public:
    Library();

    void addCategory(Category *category);

    std::vector<Category *> categories() const;
    void setCategories(const std::vector<Category *> &categories);

private:
    std::vector<Category *> m_categories;
};

#endif // LIBRARY_H
#ifndef LIBRARYPANEL_H
#define LIBRARYPANEL_H

#include <QTreeWidget>

/**
 * @brief The LibraryPanel class is the widget that goes in the left toolbar. It will hold
 * @Function s through the @Library.
 */
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
#ifndef LINK_H
#define LINK_H

#include "script.h"
#include "types.h"

#include <QGraphicsItem>
#include <QUuid>
#include <QPainterPath>
#include <QGraphicsLineItem>
#include <QString>

/**
 * @brief The Link class represents a link between neural functions (more precisely between
 * @DiagramBox es) Even more precisely, a Link is between a @DiagramBox 's @OutputSlot and
 * another (or same) @DiagramBox 's @InputSlot.
 * Depending on its type, it can have a weight.
 */

class InputSlot;
class OutputSlot;

Q_DECLARE_METATYPE(Connectivity);

class Link : public QObject, public QGraphicsItem
{
	Q_OBJECT

public:
	explicit Link(OutputSlot *f, InputSlot *t, QGraphicsItem *parent = 0);

	QRectF boundingRect() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0);
	QPainterPath shape() const;
	bool isStringLink();
	void updateTooltip();

	void addLinesToScene();
	void removeLinesFromScene();

	void updateLines();

	bool checkIfInvalid();

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	OutputSlot *from() const;
	void setFrom(OutputSlot *from);

	InputSlot *to() const;
	void setTo(InputSlot *to);

	bool secondary() const;
	void setSecondary(bool secondary);

	qreal weight() const;
	void setWeight(const qreal &weight);

	bool isInvalid() const;
	void setIsInvalid(bool isInvalid);

	bool selfLoop() const;

	QString value() const;
	void setValue(const QString &value);

	Connectivity connectivity() const;
	void setConnectivity(const Connectivity &connectivity);

	LinkInvalidReason invalidReason() const;
	void setInvalidReason(const LinkInvalidReason &invalidReason);

private:
	bool checkIfSelfLoop();

	QUuid m_uuid;           // Unique identifier
	OutputSlot *m_from;     // The OutputSlot this link goes from
	InputSlot *m_to;        // The InputSlot this link goes to
	bool m_secondary;       // Tells whether a link is a secondary link
	bool m_selfLoop;        // Tells whether a link loop back to the same function

	QGraphicsLineItem m_line;          // Main line that represents the link
	QGraphicsLineItem m_rightSegment;  // Right segment (for secondary links)
	QGraphicsLineItem m_leftSegment;   // Left segment (for secondary links)

	qreal m_weight;            // The weight associated to this link
	QString m_value;           // The string value associated to this link (when between strings)

	bool m_isInvalid; // Tells that this link is currently not valid (error in type, in sizes, etc.)
	LinkInvalidReason m_invalidReason; // Tell why a link is invalid

	Connectivity m_connectivity; // Only viable for MATRIX_MATRIX
};

#endif // LINK_H
#ifndef MATRIXFETCHER_H
#define MATRIXFETCHER_H

#include "scalarvisualization.h"

#include <QString>

#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"

class ScalarVisualization;
class MatrixVisualization;

class MatrixFetcher : public DataFetcher
{
	Q_OBJECT
public:
	explicit MatrixFetcher(const QString &topicName, MatrixVisualization *matrixVisualization, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, MatrixVisualization *matrixVisualization, VisualizationType type, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, VisualizationType type, QObject *parent = nullptr);

	void setVisType(VisualizationType type) override;

protected:
	void run() override;
	void fetchMatrix(const std_msgs::Float64MultiArray::ConstPtr& mat);

private:
	ScalarVisualization *m_scalarVisualization;
	MatrixVisualization *m_matrixVisualization;
	QList<double> m_dataList;
};

#endif // MATRIXFETCHER_H
#ifndef MATRIXVISUALIZATION_H
#define MATRIXVISUALIZATION_H

#include "datavisualization.h"
#include "rossession.h"

#include <QImage>
#include <QLabel>
#include <QVBoxLayout>


class MatrixVisualization : public DataVisualization
{
	Q_OBJECT
public:
	explicit MatrixVisualization(QWidget *parent = nullptr,
	                             ROSSession *rosSession = nullptr,
	                             QGraphicsScene *scene = nullptr,
	                             DiagramBox *box = nullptr);

private:
	QLabel *m_thermalImageLabel;
	QImage m_thermalImage;
	QVBoxLayout *m_vLayout;

private slots:
	void switchToThermal();
	void switchToImage();
	void switchToLandscape();
	void updateThermal(QList<double> *values);
};

#endif // MATRIXVISUALIZATION_H
#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <QApplication>
#include <QString>

void coloredMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg);

#endif //MESSAGE_HANDLER_H
#ifndef MOVECOMMAN_H
#define MOVECOMMAN_H

#include "diagrambox.h"

#include <QUndoCommand>

/**
 * @brief The MoveCommand class represents moving a @DiagramBox on a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */

class MoveCommand : public QUndoCommand
{
public:
	MoveCommand(DiagramBox *box, const QPointF &oldPos, QUndoCommand *parent = nullptr);
	void undo() override;
	void redo() override;

private:
	DiagramBox *m_box;
	QPointF m_oldPos;
	QPointF m_newPos;
};

#endif // MOVECOMMAN_H
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
#ifndef OUTPUTSLOT_H
#define OUTPUTSLOT_H

#include "slot.h"
#include "types.h"

#include <QString>
#include <set>

/**
 * @brief The specialized version of @Slot used for output.
 * They contain @Link s that goes out of a @DiagramBox.
 * Outputs slots react to the mouse by growing sightly when the mouse comes nears them, in order
 * to facilitate the creation of a link.
 */

class Link;

class OutputSlot : public Slot
{
    Q_OBJECT
public:
    explicit OutputSlot();

    std::set<Link *> outputs() const;

    void addOutput(Link *output);
    void removeOutput(Link *output);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QRectF boundingRect() const override;

    void mousePressEvent(QGraphicsSceneMouseEvent *evt);
    void hoverEnterEvent(QGraphicsSceneHoverEvent *evt);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *evt);

    bool isDrawingLine() const;
    void setIsDrawingLine(bool isDrawingLine);

    void updateLinks();

    OutputType outputType() const;
    void setOutputType(const OutputType &outputType);

private:
    std::set<Link *> m_outputs; // The set of links which leaves this slot
    bool m_isDrawingLine;       // Indicate if we are drawing an outgoing link
    OutputType m_outputType;    // Indicate whether this function (slot) outputs a matrix or scalar
};

#endif // OUTPUTSLOT_H
#ifndef PAPYRUSWINDOW_H
#define PAPYRUSWINDOW_H

#include "librarypanel.h"
#include "library.h"
#include "script.h"
#include "propertiespanel.h"
#include "rosnode.h"
#include "homepage.h"
#include "rossession.h"
#include "types.h"
#include "xmldescriptionreader.h"
#include "token.h"

#include <QMainWindow>
#include <QGraphicsScene>
#include <QDir>
#include <QSystemTrayIcon>
#include <QLineEdit>
#include <QLabel>
#include <QAction>
#include <QTimer>

namespace Ui {
class PapyrusWindow;
}

// Define the type of development environment (and where to go look for libraries)
enum DevelopmentType {
	RELEASE,
	DEBUG
};
Q_DECLARE_METATYPE(DevelopmentType) // This allows convertion from/to QVariant

// Define if we are asking the user for the DESCRIPTION or the LIBRARY path
enum PathType {
	PATH_DESC,
	PATH_LIB
};
Q_DECLARE_METATYPE(PathType);

/**
 * @brief The PapyrusWindow class is the main window of the application.
 * It contains the list of open @Script s, the @Library of @Function s,
 * the @DiagramView s, etc.
 */
class PapyrusWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit PapyrusWindow(int argc, char **argv, QWidget *parent = 0);
	~PapyrusWindow();

	void closeEvent(QCloseEvent *evt);

	void readSettings(QString &lastOpenedScripts, int *lastActiveScript);
	void writeSettings();
	Script *parseXmlScriptFile(const QString &scriptPath);
	void askForPath(bool displayWarning, const PathType &pathType);
	void parseOneLevel(QDir dir, XmlDescriptionReader *xmlReader);
	QString getDescriptionPath();
	QString getLibPath();
	void updateButtonsState();

	QDir description() const {return description_;}
	void setDescription(QDir description) {description_ = description;}

	Category *addTreeRoot(QString name);

	QLineEdit *librarySearchField() const;
	void setLibrarySearchField(QLineEdit *librarySearchField);

	Ui::PapyrusWindow *ui() const;

	Library *getLibrary() const;
	void setLibrary(Library *library);

	std::set<Script *> getScripts() const;
	void addScript(Script *script);

	Script *activeScript() const;

	PropertiesPanel *propertiesPanel() const;
	void setPropertiesPanel(PropertiesPanel *propertiesPanel);

	QSystemTrayIcon *getTrayIcon() const;

	RosNode *rosnode() const;
	void setRosnode(RosNode *rosnode);

	void spawnRosNode();

	HomePage *homePage() const;
	void setHomePage(HomePage *homePage);

	DevelopmentType developmentType() const;

	QString debugPath() const;

	QString releasePath() const;

	QString debugLibPath() const;

	QString releaseLibPath() const;

	QString keyFile() const;
	void setKeyFile(const QString &keyFile);

	QString ivFile() const;
	void setIvFile(const QString &ivFile);

	QString lastDir() const;
	void setLastDir(const QString &lastDir);

	QTimer *autoSaveTimer() const;
	void setAutoSaveTimer(QTimer *autoSaveTimer);

private:
	Ui::PapyrusWindow *m_ui;
	RosNode *m_rosnode;
	int m_argc;
	char **m_argv;
	QLabel *m_rosMasterStatus;
	LibraryPanel *libraryPanel_;
	QLineEdit *librarySearchField_;
	QString m_lastExpandedCategory;  // Name of the last category that was expanded before filtering
	int m_libraryParsingErrors;
	QDir description_;
	QSystemTrayIcon *trayIcon;
	Library *m_library;
	std::set<Script *> m_scripts;
	Script *m_activeScript;
	PropertiesPanel *m_propertiesPanel;
	HomePage *m_homePage;
	QLineEdit *m_runTimeDisplay;
	DevelopmentType m_developmentType;
	QAction *m_actionRelease;
	QAction *m_actionDebug;
	QString m_debugPath;        // Path where to search for description files in DEBUG mode
	QString m_releasePath;      // Path where to search for description files in RELEASE mode
	QString m_debugLibPath;     // Path where to search for library files in DEBUG mode
	QString m_releaseLibPath;   // Path where to search for library files in RELEASE mode
	QString m_keyFile;          // Path of the key file to crypt / decrypt scrip files
	QString m_ivFile;           // Path of the IV
	QString m_lastDir;          // Last directory visited for saving or loading files
	QTimer *m_autoSaveTimer;    // Timer to trigger auto save for scripts
	QString m_changelogVersion; // Used to know if we should show the changelog on launch
	QTimer *m_checkVersionTimer; // Timer that periodically check for new version release
	bool m_preventROSPopup;     // Prevents displaying ROS master pop-ups

signals:
	void toggleDisplayGrid(bool);
	void launched();

private slots:
	void filterLibraryNames(const QString &text);
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void onROSMasterChange(bool isOnline);
	void onScriptResumed();
	void onScriptPaused();
	void onScriptStopped();
	void updateStopWatch(int h, int m, int s, int ms);
	void updateDevelopmentEnvironment(QAction *action);
	void categoryExpanded(QTreeWidgetItem *item);
	void autoSave();
	void onPropPanelEnter();
	void onPropPanelEscape();
	void onLaunched();
	void openScript(QString path = "");
	void checkForNewRelease();
	void reEnableROSPopUp();

	void on_actionExit_triggered();

	void on_actionAntialiasing_toggled(bool antialiasing);

	void on_actionZoom_In_triggered();

	void on_actionZoom_Out_triggered();

	void on_actionZoom_Fit_triggered();

	void on_actionNew_script_hovered();

	void on_actionOpen_Script_hovered();

	void on_actionSave_Script_hovered();

	void on_actionZoom_In_hovered();

	void on_actionZoom_Out_hovered();

	void on_actionZoom_Fit_hovered();

	void on_actionNew_script_triggered();

	void on_actionDisplay_Grid_hovered();

	void on_actionDisplay_Grid_toggled(bool arg1);

	void on_actionAbout_Papyrus_triggered();

	void on_actionSave_Script_triggered();

	void on_actionOpen_Script_triggered();
	void on_tabWidget_currentChanged(int index);
	void on_tabWidget_tabBarDoubleClicked(int index);
	void on_actionClose_Script_triggered();
	void on_actionConnect_triggered();
	void on_actionRun_triggered();
	void on_actionStop_triggered();
	void on_actionScope_triggered();
	void on_actionEdit_paths_triggered();
	void on_actionShow_all_outputs_triggered();
	void on_actionHide_all_outputs_triggered();
	void on_actionList_shortcuts_triggered();
	void on_actionChangelog_triggered(bool isNewRelease = false);
	void on_actionReopen_last_scripts_triggered();
	void on_actionUndo_triggered();
	void on_actionRedo_triggered();
};

#endif // PAPYRUSWINDOW_H
#ifndef PROPDOUBLESPINBOX_H
#define PROPDOUBLESPINBOX_H

#include <QDoubleSpinBox>

/**
 * @brief The PropDoubleSpinBox class is the standard QDoubleSpinBox with only one minor
 * modification: its sizeHint has been made editable
 */

class PropDoubleSpinBox : public QDoubleSpinBox
{
public:
	PropDoubleSpinBox(QWidget *parent = nullptr);

	QSize sizeHint() const override;
	void setSizeHint(const QSize &sizeHint);

private:
	QSize m_sizeHint;
};

#endif // PROPDOUBLESPINBOX_H
#ifndef PROPERTIESPANEL_H
#define PROPERTIESPANEL_H

#include "diagrambox.h"
#include "link.h"
#include "script.h"
#include "zone.h"
#include "setcolorbutton.h"
#include "proplineedit.h"
#include "propdoublespinbox.h"

#include <QGroupBox>
#include <QLineEdit>
#include <QFrame>
#include <QLabel>
#include <QSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QFormLayout>

/**
 * @brief The PropertiesPanel class is used to display (and modify) the properties of selected
 * objects (such as slots, boxes, links, etc.)
 */

class PropertiesPanel : public QGroupBox
{
	Q_OBJECT

public:
	explicit PropertiesPanel(QWidget *parent = 0);

	void hideAllFrames(bool buttonsToo = false);
	void updateBoxProperties(DiagramBox *box);
	void updateLinkProperties(Link *link);
	void updateScriptProperties(Script *script);
	void updateZoneProperties(Zone *zone);

	void keyPressEvent(QKeyEvent *event);

	QFrame *boxFrame() const;
	void setBoxFrame(QFrame *boxFrame);

	QFrame *linkFrame() const;
	void setLinkFrame(QFrame *linkFrame);

	QLabel *boxName() const;
	void setBoxName(QLabel *boxName);

	QSpinBox *rowsInput() const;

	QSpinBox *colsInput() const;

	QPushButton *okBtn() const;
	void setOkBtn(QPushButton *okBtn);

	QPushButton *cancelBtn() const;
	void setCancelBtn(QPushButton *cancelBtn);

	QCheckBox *saveActivity() const;
	void setSaveActivity(QCheckBox *saveActivity);

	QDoubleSpinBox *linkWeight() const;

	QLabel *scriptName() const;
	void setScriptName(QLabel *scriptName);

	PropDoubleSpinBox *timeValue() const;
	void setTimeValue(PropDoubleSpinBox *timeValue);

	QComboBox *timeUnit() const;
	void setTimeUnit(QComboBox *timeUnit);

	QLineEdit *linkValue() const;
	void setLinkValue(QLineEdit *linkValue);

	QPushButton *displayVisu() const;
	void setDisplayVisu(QPushButton *displayVisu);

	PropLineEdit *zoneTitle() const;
	void setZoneTitle(PropLineEdit *zoneTitle);

	SetColorButton *zoneColor() const;
	void setZoneColor(SetColorButton *zoneColor);

	PropLineEdit *boxTitle() const;
	void setBoxTitle(PropLineEdit *boxTitle);

	QLabel *boxMatrixShape() const;
	void setBoxMatrixShape(QLabel *boxMatrixShape);

	QCheckBox *publish() const;
	void setPublish(QCheckBox *publish);

	PropLineEdit *topic() const;
	void setTopic(PropLineEdit *topic);

	QCheckBox *linkSecondary() const;
	void setLinkSecondary(QCheckBox *linkSecondary);

	QCheckBox *encrypt() const;
	void setEncrypt(QCheckBox *encrypt);

private:
	QVBoxLayout *m_panelLayout;  // The properties panel's main layout
	QFrame *m_scriptFrame;       // Container for script's properties
	QLabel *m_scriptName;        // Label used to change the script (and tab) name
	QLabel *m_timeLabel;         // Contains either "frequency" or "period"
	PropDoubleSpinBox *m_timeValue; // Used to input the script's frequency (or period)
	QComboBox *m_timeUnit;       // Used to select the unit (in Hz or ms)
	QCheckBox *m_encrypt;        // Whether or not the file is encrypted on save

	QFormLayout *m_boxLayout;  // Layout for the box properties (access needed to hide rows)
	QFrame *m_boxFrame;        // Container for box's properties
	QLabel *m_boxName;         // Display the name of the box
	PropLineEdit *m_boxTitle;     // Allow to see or change the box's custom name
	QLabel *m_boxOutputType;   // Display the box's output type (scalar, matrix)
	QLabel *m_boxMatrixShape;  // Display the shape of the function (when matrix)
	QSpinBox *m_rowsInput;     // Spin box to input number of rows in the output (if matrix)
	QSpinBox *m_colsInput;     // Spin box to input number of columns in the output (if matrix)
	QCheckBox *m_saveActivity; // To enable saving the activity of the box
	QCheckBox *m_publish;      // To enable publish the output of the function
	PropLineEdit *m_topic;        // To input the topic name for publishing
	QPushButton *m_displayVisu; // (TEMP) display the box's data vizualisation

	QFormLayout *m_linkLayout;    // Layout for the link properties (access needed to hide rows)
	QFrame *m_linkFrame;          // Container for link's properties
	QLabel *m_linkType;           // Display the type of the link
	QCheckBox *m_linkSecondary;   // Will display if the link is secondary or not
	QDoubleSpinBox *m_linkWeight; // Spin box to set the weight of the link
	QLineEdit *m_linkValue;       // Text field to enter the link's value (for string links)

	QFormLayout *m_zoneLayout;     // Contains the layout to display comment zone's properties
	QFrame *m_zoneFrame;           // Container for zone's properties
	PropLineEdit *m_zoneTitle;        // The comment zone's title
	SetColorButton *m_zoneColor;     // Holds the color of the comment zone

	// Will contain the sizes of connected matrix when the link is SPARSE_MATRIX
	QSize m_inputSize;
	QSize m_outputSize;

	QPushButton *m_okBtn;      // Button used to validate changes in parameters
	QPushButton *m_cancelBtn;  // Button used to discard changes in parameters and restore current

public slots:
	void displayBoxProperties(DiagramBox *box);
	void displayLinkProperties(Link *link);
	void displayScriptProperties(Script *script);
	void displayZoneProperties(Zone *zone);
	void convertTimeValues(int idx);
	void toggleTopic(bool isChecked);

private slots:
	void onTopicChanged(const QString &topic);

signals:
	void enterPressed();
	void escapePressed();
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
};

#endif // PROPERTIESPANEL_H
#ifndef PROPLINEEDIT_H
#define PROPLINEEDIT_H

#include <QLineEdit>

/**
 * @brief The PropLineEdit class is a standard @QLineEdit with only one minor modification: its
 * sizeHint can be changed
 */

class PropLineEdit : public QLineEdit
{
public:
	PropLineEdit(QWidget *parent = nullptr);
	PropLineEdit(const QString &contents, QWidget *parent = nullptr);

	QSize sizeHint() const override;
	void setSizeHint(const QSize &sizeHint);

private:
	QSize m_sizeHint; // Modified sizeHint
};

#endif // PROPLINEEDIT_H
#ifndef ROSNODE_H
#define ROSNODE_H

#include <QThread>

#include <ros/ros.h>

/**
 * @brief The RosNode class is used to handle ROS
 * operations
 */
class RosNode : public QThread
{
	Q_OBJECT
public:
	RosNode(int argc, char **argv);
	virtual ~RosNode();

	void init();
	void run() override;

	bool shouldQuit() const;
	void setShouldQuit(bool value);

private:
	int m_argc;
	char **m_argv;
	ros::Subscriber m_sub;
	bool m_shouldQuit;       // Used by Papyrus to cleanly exit the thread

signals:
	void rosMasterChanged(bool isOnline);
};

#endif // ROSNODE_H
#ifndef ROSSESSION_H
#define ROSSESSION_H

#include <ros/ros.h>

//#include "script.h"
#include "types.h"

#include <QThread>
#include <QString>
#include <QSet>

#include "diagnostic_msgs/KeyValue.h"

/**
 * @brief The ROSSession class contains parameters related to the current ROS session (connection
 * with a Kheops node
 */
class ROSSession : public QThread
{
	Q_OBJECT

public:
	ROSSession(const QString &nodeName, QObject *parent = nullptr);
	~ROSSession();

	void addToHotList(QUuid uuid);

	bool shouldQuit() const;
	void setShouldQuit(bool shouldQuit);

	QString nodeName() const;
	void setNodeName(const QString &nodeName);

private:
	bool m_shouldQuit;       // Used to properly exit the thread
	QString m_nodeName;      // Name of the node it should listen
	QSet<QUuid> m_hotList;   // List of functions' uuids whose output to activate on-the-fly
	bool m_isFirstRun;       // To differentiate a 'resume' on first launch or after a pause

	void run() override;
	void handleStatusChange(const diagnostic_msgs::KeyValue::ConstPtr& msg);
	void activateOutput(QUuid uuid);

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void scriptResumed();
	void scriptPaused();
	void scriptStopped();
};

#endif // ROSSESSION_H
#ifndef SCALARFETCHER_H
#define SCALARFETCHER_H

#include "datafetcher.h"
#include "types.h"

#include <QDebug>
#include <QBarSet>
#include <QSplineSeries>

#include <ros/ros.h>
#include "std_msgs/Float64.h"

QT_CHARTS_USE_NAMESPACE

class ScalarVisualization;

class ScalarFetcher : public DataFetcher
{
	Q_OBJECT
public:
	explicit ScalarFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent = nullptr);
	explicit ScalarFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, VisualizationType type, QObject *parent = nullptr);

	void setVisType(VisualizationType type) override;

protected:
	void run() override;

	void fetchScalar(const std_msgs::Float64::ConstPtr& scalar);

private:
	ScalarVisualization *m_scalarVisualization;
};

#endif // SCALARFETCHER_H
#ifndef SCALARVISUALIZATION_H
#define SCALARVISUALIZATION_H

#include "types.h"
#include "datavisualization.h"
#include "rossession.h"

#include <QBarSet>
#include <QBarSeries>
#include <QHorizontalBarSeries>
#include <QChart>
#include <QChartView>
#include <QSplineSeries>
#include <QValueAxis>
#include <QVBoxLayout>
#include <QList>

#include <vector>

QT_CHARTS_USE_NAMESPACE

class ScalarVisualization : public DataVisualization
{
	Q_OBJECT
public:
	ScalarVisualization(QWidget *parent = nullptr,
	                    ROSSession *rosSession = nullptr,
	                    QGraphicsScene *scene = nullptr,
	                    DiagramBox *box = nullptr);
	void mousePressEvent(QMouseEvent *evt);

	void updateBarValues(const std::vector<qreal> &values);
	void pushGraphValues(const std::vector<qreal> &values);

protected:
	void createCharts();

	int m_size;
	int m_idx;

	QBarSet *m_barSet;
	QBarSeries *m_barSeries;
	QHorizontalBarSeries *m_horizontalBarSeries;
	QChart *m_barChart;
	QValueAxis *m_barAxisY;
	QValueAxis *m_barAxisX;
	QChartView *m_barView;
	qreal m_barMin;
	qreal m_barMax;
	MatrixShape m_matrixShape;  // used to determine wether to display a ROW or COL vector (when applicable)

	QList<QSplineSeries *> m_graphSeries;
	QChart *m_graphChart;
	QVBoxLayout *m_vLayout;
	QValueAxis *m_graphAxisX;
	QValueAxis *m_graphAxisY;
	QChartView *m_graphView;
	qreal m_graphMin;
	qreal m_graphMax;

protected slots:
	void switchToBar();
	void switchToGraph();
};

#endif // SCALARVISUALIZATION_H
#ifndef SCRIPT_H
#define SCRIPT_H

#include "diagramscene.h"
#include "types.h"
#include "rossession.h"

#include <QString>
#include <QFile>
#include <QDir>
#include <QUuid>
#include <QTimer>

enum ScriptStatus {
	INVALID_SCRIPT_STATUS,
	SCRIPT_RUNNING,
	SCRIPT_PAUSED
};

// Forward declaration because of recursive includes
class DiagramScene;

Q_DECLARE_METATYPE(TimeUnit) // This allows convertion from/to QVariant

/**
 * @brief The Script class represents a neural scripts. An XML file is associated to it,
 * as well as the @QGraphicsScene that contains its functions boxes
 */

class Script : public QObject
{
	Q_OBJECT
public:
	Script(DiagramScene *scene, const QString &name = "");
	~Script();

	void save(const QString &basePath = QDir::homePath(),
	          bool isAutoSave = false);

	void updateTextStyle();
	void runOrPause();
	void run();
	void pause();
	void stop();
	ScriptStatus queryScriptStatus();
	void setupROSSession();

	QString name() const;
	void setName(const QString &name);

	QString filePath() const;
	void setFilePath(const QString &filePath);

	DiagramScene *scene() const;

	bool modified() const;

	void setStatusModified(bool isModified);

	bool isInvalid() const;
	void setIsInvalid(bool isInvalid);

	double timeValue() const;
	void setTimeValue(double timeValue);

	TimeUnit timeUnit() const;
	void setTimeUnit(const TimeUnit &timeUnit);

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	bool encrypt() const;
	void setEncrypt(bool encrypt);

	ROSSession *rosSession() const;
	void setRosSession(ROSSession *rosSession);

	bool isActiveScript() const;
	void setIsActiveScript(bool isActiveScript);

	bool hasTab() const;
	void setHasTab(bool hasTab);

	bool isRunning() const;
	void setIsRunning(bool isRunning);

	bool isPaused() const;
	void setIsPaused(bool isPaused);

	QString nodeName() const;
	void setNodeName(const QString &nodeName);

public slots:
	void warnAboutModifiedScript();

private:
	DiagramScene *m_scene; // The associated scene for this script
	bool m_hasTab; // Tells whether the scripts has a tab in the tabwidget or not
	ROSSession *m_rosSession; // The associated ROS Session for this script
	QString m_name;        // Pretty name of the script (to display in tabs for instance)
	QString m_nodeName;    // The ROS node name of the script
	QString m_filePath;    // Path of the (XML) file in which to save this script
	bool m_modified;       // Whether there was some changes since last save
	bool m_isInvalid;      // Whether this script is currently invalid (and thus prevent saving)
	double m_timeValue;    // The RT Token time (either frequency or period)
	TimeUnit m_timeUnit;   // Whether the time value is a frequency or a period
	QUuid m_uuid;          // UUID for the RT Token (needed by kheops)
	QTimer *m_modifiedNotifTimer; // Timer to display a system tray notification when unsaved for more than X minutes
	bool m_encrypt;        // Whether the XML script should be encrypted on save (to protect IP)
	std::string m_key;     // AES Key used to encrypt the file
	std::string m_iv;      // AES IV used to encrypt the file
	bool m_isActiveScript; // Tells this script if it's the currently active one
	bool m_isRunning;      // Tells whether this script is running (launched)
	bool m_isPaused;       // Tells whether this script is paused while running

private slots:
	void onROSSessionMessage(const QString &msg, MessageUrgency urgency = MSG_INFO);
	void onScriptResumed();
	void onScriptPaused();
	void onScriptStopped();
	void onTimeElapsed(int h, int m, int s, int ms);
//	void temporaryCheckLaunch();

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void scriptResumed();
	void scriptPaused();
	void scriptStopped();
	void timeElapsed(int h, int m, int s, int ms);
};

#endif // SCRIPT_H
#ifndef SETCOLORBUTTON_H
#define SETCOLORBUTTON_H

#include <QPushButton>
#include <QColor>

class SetColorButton : public QPushButton
{
	Q_OBJECT

public:
	explicit SetColorButton(QWidget *parent = nullptr);

	void updateColor();

	QColor color() const;
	void setColor(const QColor &color);

private:
	QColor m_color;

private slots:
	void changeColor();
};

#endif // SETCOLORBUTTON_H
#ifndef SLOT_H
#define SLOT_H

#include <QObject>
#include <QString>
#include <QGraphicsItem>

//#include <diagrambox.h>
class DiagramBox;

/**
 * @brief The Slot class defines an argument slot, i.e. an item will either receive
 * a connection from another box's output slot, or from which a connection leaves to
 * reach another box' input slot.
 * This class is meant to be subclassed (see @InputSlot and @OutputSlot).
 */

/*
enum InputType {
    SCALAR_SCALAR,
    SCALAR_MATRIX,
    MATRIX_MATRIX,
    SPARSE_MATRIX
};

enum OutputType {
    SCALAR,
    MATRIX
};
*/

class Slot : public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    explicit Slot(QGraphicsItem *parent = 0);
    explicit Slot(QString &name, QGraphicsItem *parent = 0);
    ~Slot();

    QString name() const;
    void setName(const QString &name);

    virtual QRectF boundingRect() const = 0;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) = 0;

    qreal dist() const;
    void setDist(const qreal &dist);

    DiagramBox *box() const;
    void setBox(DiagramBox *box);

protected:
    QString m_name;    // The name of this slot
    qreal m_dist;      // Distance to the mouse (used to highlight the slot when mouse approach)
    DiagramBox *m_box; // The DiagramBox that is associated with this Slot
};

#endif // SLOT_H
#ifndef SWAPBOXESCOMMAND_H
#define SWAPBOXESCOMMAND_H

#include "diagrambox.h"
#include "diagramscene.h"
#include "link.h"

#include <QUndoCommand>

/**
 * @brief The SwapBoxesCommand class represents swapping two functions. This is used to provide
 * Undo/Redo functionality.
 */

class SwapBoxesCommand : public QUndoCommand
{
public:
	SwapBoxesCommand(DiagramScene *scene, DiagramBox *toSwap, DiagramBox *newBox,
	                 QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	DiagramScene *m_scene;  // The scene in which the swap takes place
	DiagramBox *m_toSwap;   // The box to be swapped
	DiagramBox *m_newBox;   // The box to add

	QList<Link *> m_outputLinks; // List of all links that are FROM m_toSwap
	QHash<QString, QList<Link *>> m_inputLinks; // List of all links that are TO m_toSwap, sorted by input slots
};

#endif // SWAPBOXESCOMMAND_H
#ifndef TOKEN_H
#define TOKEN_H

// Define token and password to check Papyrus version on gitlab
#define GITLAB_TOKEN "gitlab+deploy-token-1"
#define GITLAB_PWD "xz7LJDr-bMe6wZYHzLze"

#endif // TOKEN_H
#ifndef TYPES_H
#define TYPES_H

/**
 * This is just a collection of types (no functions!) in order to simply the include process
 */

// Define the type that an @InputSlot accepts
enum InputType{
	INVALID_INPUT_TYPE,
	SCALAR_SCALAR,
	SCALAR_MATRIX,
	MATRIX_MATRIX,
	STRING_INPUT
};

// Define the type that an @OutputSlot (hence a function, really) outputs
enum OutputType{
	INVALID_OUTPUT_TYPE,
	SCALAR,
	MATRIX,
	STRING
};

// Define the shape of a matrix-typed @DiagramBox
enum MatrixShape {
	INVALID_MATRIX_SHAPE,
	SHAPE_NONE,
	POINT,
	VECT,
	ROW_VECT,
	COL_VECT
};

// Define whether the time value is a frequency in Hz or a period in ms for a @Script
enum TimeUnit{
	INVALID_UNIT,
	HZ,
	MS
};

// Define the urgency of a message that will be shown on the status bar (and then its color)
enum MessageUrgency {
	INVALID_MESSAGE_URGENCY,
	MSG_INFO,
	MSG_WARNING,
	MSG_ERROR
};

// Define the type of connectivity for MATRIX_MATRIX links
enum Connectivity {
	INVALID_CONNECTIVITY,
	ONE_TO_ONE,
	ONE_TO_ALL,
	ONE_TO_NEI
};

// Define the type of visualization for data
enum VisualizationType {
	INVALID_VISUALIZATION_TYPE,
	BAR,        // scalar and vector
	GRAPH,      // scalar and vector
	IMAGE,      // matrix
	GRAYSCALE,  // matrix
	LANDSCAPE   // matrix
};

// Define the different reasons why a Link can be invalid
enum LinkInvalidReason {
	INVALID_INVALID_REASON = 0, // MUST be zero (because it uses bitwise OR)
	TYPES_INCOMPATIBLE     = 0b1,
	SIZES_DONT_MATCH       = 0b10,
	SHAPE_MUST_BE_POINT    = 0b100,
	SHAPE_MUST_BE_VECT     = 0b1000,
	SHAPE_MUST_BE_ROW_VECT = 0b10000,
	SHAPE_MUST_BE_COL_VECT = 0b100000
};

// Defining bitwise operations for LinkInvalidReason because in C++11, enums are scoped
inline LinkInvalidReason operator|(LinkInvalidReason a, LinkInvalidReason b) {
	return static_cast<LinkInvalidReason>(static_cast<int>(a) | static_cast<int>(b));
}

inline LinkInvalidReason operator&(LinkInvalidReason a, LinkInvalidReason b) {
	return static_cast<LinkInvalidReason>(static_cast<int>(a) & static_cast<int>(b));
}

// Define the different reasons why a box can be invalid
enum BoxInvalidReason {
	INVALID_BOX_INVALID_REASON = 0, // MUST be zero (because it uses bitwise OR)
	INPUT_FULL                 = 0b1,
	BOX_MUST_BE_POINT          = 0b10,
	BOX_MUST_BE_VECT           = 0b100,
	BOX_MUST_BE_ROW_VECT       = 0b1000,
	BOX_MUST_BE_COL_VECT       = 0b10000,
};

// Defining bitwise operations for BoxInvalidReason because in C++11, enums are scoped
inline BoxInvalidReason operator|(BoxInvalidReason a, BoxInvalidReason b) {
	return static_cast<BoxInvalidReason>(static_cast<int>(a) | static_cast<int>(b));
}

inline BoxInvalidReason operator&(BoxInvalidReason a, BoxInvalidReason b) {
	return static_cast<BoxInvalidReason>(static_cast<int>(a) & static_cast<int>(b));
}

#endif // TYPES_H
#ifndef UPDATEBOXCOMMAND_H
#define UPDATEBOXCOMMAND_H

#include "propertiespanel.h"
#include "diagrambox.h"

#include <QUndoCommand>

/**
 * @brief The UpdateZoneCommand class represents updating a @DiagramBox's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateBoxCommand : public QUndoCommand
{
public:
	UpdateBoxCommand(PropertiesPanel *panel, DiagramBox *box, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	DiagramBox *m_box;        // The box which we are modifying parameters

	// Old parameters (ones the box had before updating its parameters)
	QString m_oldTitle;
	int m_oldRows;
	int m_oldCols;
	bool m_oldActivity;
	bool m_oldPublish;
	QString m_oldTopic;

	// New parameters (ones the box will have after updating its parameters)
	QString m_newTitle;
	int m_newRows;
	int m_newCols;
	bool m_newActivity;
	bool m_newPublish;
	QString m_newTopic;
};

#endif // UPDATEBOXCOMMAND_H
#ifndef UPDATELINKCOMMAND_H
#define UPDATELINKCOMMAND_H

#include "propertiespanel.h"
#include "link.h"

#include <QUndoCommand>

/**
 * @brief The UpdateLinkCommand class represents updating a @Link's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateLinkCommand : public QUndoCommand
{
public:
	UpdateLinkCommand(PropertiesPanel *panel, Link *link, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	Link *m_link;             // The link which we are modifying parameters

	// Old parameters
	qreal m_oldWeight;
	QString m_oldValue;
	bool m_oldSecondary;

	// New parameters
	qreal m_newWeight;
	QString m_newValue;
	bool m_newSecondary;
};

#endif // UPDATELINKCOMMAND_H
#ifndef UPDATESCRIPTCOMMAND_H
#define UPDATESCRIPTCOMMAND_H

#include "propertiespanel.h"
#include "script.h"

#include <QUndoCommand>

/**
 * @brief The UpdateScriptCommand class represents updating a @Script's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateScriptCommand : public QUndoCommand
{
public:
        UpdateScriptCommand(PropertiesPanel *panel, Script *script, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	Script *m_script;           // The script which we are modifying parameters

	// Old parameters
	qreal m_oldTime;
	TimeUnit m_oldUnit;
	bool m_oldEncrypt;

	// New parameters
	qreal m_newTime;
	TimeUnit m_newUnit;
	bool m_newEncrypt;
};

#endif // UPDATESCRIPTCOMMAND_H
#ifndef UPDATEZONECOMMAND_H
#define UPDATEZONECOMMAND_H

#include "propertiespanel.h"
#include "zone.h"

#include <QUndoCommand>

/**
 * @brief The UpdateZoneCommand class represents updating a @Zone's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateZoneCommand : public QUndoCommand
{
public:
	UpdateZoneCommand(PropertiesPanel *panel, Zone *zone, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	Zone *m_zone;             // The zone which we are modifying parameters

	// Old parameters
	QString m_oldTitle;
	QColor m_oldColor;

	// New parameters
	QString m_newTitle;
	QColor m_newColor;
};

#endif // UPDATEZONECOMMAND_H
#ifndef XMLDESCRIPTIONREADER_H
#define XMLDESCRIPTIONREADER_H

#include "category.h"
#include "slot.h"

#include <QXmlStreamReader>

/**
 * @brief The XmlDescriptionReader class is a XML file parser whose task is to populate a @Library
 * object with the @Categorie  and descriptions of neural @Function s, read from their XML description
 * file.
 */

class XmlDescriptionReader
{
public:
	XmlDescriptionReader(Category *category );

	bool read(QIODevice *device, const QString &descriptionFile);
private:
//    Library *m_library;
	Category *m_category;
	QXmlStreamReader reader;

	void readDescription(const QString &descriptionFile);
	void readAllFunctions(const QString &libName, const QString &descriptionFile);
	void readOneFunction(const QString &libName, const QString &descriptionFile);
	void readName(Function *function);
	void readInputs(Function *function);
	QString readIcon();
	void readParameterName(Slot *paramSlot);
	void readParameterType(OutputSlot *paramSlot);
	void readParameterType(InputSlot *paramSlot);
	void readParameterDesc(InputSlot *paramSlot);
	void readOutput(Function *function);
	void readFunctionDesc(Function *function);
};

#endif // XMLDESCRIPTIONREADER_H
#ifndef XMLSCRIPTREADER_H
#define XMLSCRIPTREADER_H

#include "script.h"
#include "diagrambox.h"
#include "outputslot.h"
#include "inputslot.h"
#include "library.h"

#include <QXmlStreamReader>
#include <QUuid>

/**
 * @brief The XmlScriptReader class is an XML parser that reads an XML @Script file
 * to populate the @GraphicsScene.
 */

class XmlScriptReader
{
public:
	explicit XmlScriptReader(Script *script, const QString &descriptionPath, Library *library);
	bool read(QIODevice *device);

	QString errorString() const;

	QPointF centerView() const;
	void setCenterView(const QPointF &centerView);

	Library *library() const;
	void setLibrary(Library *library);

private:
	QXmlStreamReader reader;
	QString m_errorString;
	Script *m_script;
	QString m_descriptionPath;
	QPointF m_centerView;
	Library *m_library;

	void readScript();
	void readFunction(std::map<QUuid, DiagramBox *> *allBoxes,
	                  std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readFunctionName(QString &name);
	void readFunctionTitle(QString &title);
	void readFunctionSave(bool *save);
	void readPublishTopic(QString &topic, bool *publish);
	void readInputSlots(std::vector<InputSlot *> *inputSlots,
	                    std::map<QUuid, DiagramBox *> *allBoxes,
	                    std::set<std::pair<QUuid, Link *>> *incompleteLinks);
	void readOutputSlot(OutputSlot *outputSlot, int *rows, int *cols);
	void readUUID(QUuid *uuid);
	void readPosition(QPointF *pos);
	void readLinks(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
	               std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readLink(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
	              std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readZone();
};

#endif // XMLSCRIPTREADER_H
#ifndef ZONE_H
#define ZONE_H

#include <QGraphicsObject>
#include <QGraphicsItem>
#include <QColor>
#include <QGraphicsItemGroup>

enum ResizeType {
	NO_RESIZE,
	RESIZE_TOP,
	RESIZE_RIGHT,
	RESIZE_BOTTOM,
	RESIZE_LEFT
};

class Zone : public QGraphicsObject
{
public:
	explicit Zone(QGraphicsObject *parent = nullptr);
	explicit Zone(qreal x, qreal y, qreal w, qreal h, QGraphicsObject *parent = nullptr);
	~Zone();

	QRectF boundingRect() const override;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	QVariant itemChange(GraphicsItemChange change, const QVariant &value);
	void mousePressEvent(QGraphicsSceneMouseEvent *event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	void hoverMoveEvent(QGraphicsSceneHoverEvent *event);

	void updateGroup(bool startFromScratch = false);
	void updateLinks();

	// Getters / Setters

	qreal width() const;
	void setWidth(const qreal &width);

	qreal height() const;
	void setHeight(const qreal &height);

	QColor color() const;
	void setColor(const QColor &color);

	QString title() const;
	void setTitle(const QString &title);

private:
	qreal m_width;
	qreal m_height;
	QColor m_color;
	QString m_title; // The title of the comment zone (should be kept small)
	ResizeType m_resizeType;
};

#endif // ZONE_H
#ifndef ADDBOXCOMMAND_H
#define ADDBOXCOMMAND_H

#include "diagrambox.h"
#include "diagramscene.h"

#include <QUndoCommand>

/**
 * @brief The AddBoxCommand class represents adding a @DiagramBox to aa @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddBoxCommand : public QUndoCommand
{
public:
	AddBoxCommand(DiagramScene *scene, DiagramBox *box, const QPointF &initialPos, QUndoCommand *parent = nullptr);
	~AddBoxCommand();

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene; // The scene in which the box should be added
	DiagramBox *m_box;     // The DiagramBox to add to the scene
	QPointF m_initialPos;  // The initial position at which the box is added
};

#endif // ADDBOXCOMMAND_H
#ifndef ADDLINKCOMMAND_H
#define ADDLINKCOMMAND_H

#include "diagramscene.h"
#include "link.h"
#include "inputslot.h"
#include "outputslot.h"

#include <QUndoCommand>

/**
 * @brief The AddLinkCommand class represents adding a @Link to a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddLinkCommand : public QUndoCommand
{
public:
	AddLinkCommand(DiagramScene *scene, Link *link, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene; // The scene in which the link is added
	Link *m_link;          // The link to add to the scene
	OutputSlot *m_from;    // The output slot the link is from
	InputSlot *m_to;       // The input slot the link goes to
	bool m_isFirst;        // Flag that indicates wheather a redo() action is the first one
};

#endif // ADDLINKCOMMAND_H
#ifndef ADDZONECOMMAND_H
#define ADDZONECOMMAND_H

#include "diagramscene.h"
#include "zone.h"

#include <QUndoCommand>

/**
 * @brief The AddZoneCommand class represents adding a @Zone to a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddZoneCommand : public QUndoCommand
{
public:
	AddZoneCommand(DiagramScene *scene, Zone *zone, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene;  // The scene in which to add the zone
	Zone *m_zone;           // The zone to add to the scene
};

#endif // ADDZONECOMMAND_H
#ifndef CATEGORY_H
#define CATEGORY_H

#include "function.h"

#include <vector>
#include <QTreeWidgetItem>

/**
 * @brief The Category class holds a number of neural @Function 's descriptions that are part of the
 * same category.
 * This is used with the @Library and the @LibraryPanel. The idea is to group similar functions
 * by themes. The categories are simply made by parsing the directory name in which the function's
 * descriptions are saved.
 */

class Category : public QTreeWidgetItem
{
public:
    Category(const QString &name);

    QString name() const;
    void setName(const QString &name);

private:
    QString m_name;
};

#endif // CATEGORY_H
#ifndef CHANGELOG_H
#define CHANGELOG_H

#include <QString>

QString changelog = "<h3>CHANGELOG</h3>"
                    "<ul>"

                    "<li><strong>v0.5.1</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>When saving the same script, without adding any boxes, and just making "
                    "position changes, or changing the weight of a link, the order in which "
                    "functions and links were written in the XML was not deterministic. It has been "
                    "fixed, and now both functions and links are written in order of their UUID. "
                    "Moving boxes or changing link weights will produce only a small git diff for "
                    "the coordinates / weights.</li>"
                    "<li>"
                    "<li>There was a segfault when closing Papyrus after opening two scripts. This "
                    "prevented the normal cleaning procedures to happen and thus settings to be "
                    "saved (among other things). This is now fixed.</li>"
                    "<li>The libraries in the library panel on the left are not sorted "
                    "alphabetically (they were sorted in the reverse order)</li>"
                    "<li>Lots of XML information about the function boxes were saved in the script's"
                    " XML and thus any changed in alexandria (such as the icon, the name of inputs, "
                    "etc.) were not being reloaded. Now only the minimum information is stored in "
                    "the XML, instead, much of the information regarding a box are being re-parsed "
                    "from the library when a script is opened.</li>"
                    "<li>There was a compilation error on older Qt version, because a QStringRef "
                    "could not be used in the same manner, this was converted back to QString to fix"
                    " this.</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.5.0</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Parse <description> tags from XML description files and display a "
                    "function's description (and its input) when hovering the library</li>"
                    "<li>Fix the graphical glitch where a link seemed to \"go out in the distance\" "
                    "when a Comment Zone was resized and the box was now out of the zone</li>"
                    "<li>The full version MAJOR.MINOR.BUGFIX is now displayed in the Home Page and "
                    "the About dialog (the bugfix version was missing) which allows the user to know"
                    " which version he is running</li>"
                    "<li>Only one Comment Zone was parsed & loaded when a script was opened, this is"
                    " now fixed</li>"
                    "<li>Function boxes used to be transparent which made them weird-looking when "
                    "placed inside Comment Zones, now their background is white</li>"
                    "<li><strong>(new feature)</strong> Papyrus now periodically checks the gitlab repository for a "
                    "new release and when it finds one, it warns the user with a dialog</li>"
                    "<li><strong>(new feature)</strong> Papyrus will now re-open with the last "
                    "opened scripts (and the last active script) by default. It is possible, however"
                    " to disable this feature in the options</li>"
                    "<li><strong>(new feature)</strong> The full CHANGELOG is now visible in the "
                    "Help > CHANGELOG menu. Also, when it's the first time a new version is "
                    "launched, the CHANGELOG is automatically displayed.</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.4.2</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Fix the big issue when events on properties panel (such as clicking OK, CANCEL, "
                    "or pressing ENTER) was propagated to all open scenes. It was thus unsafe to edit "
                    "several scripts at the same time. Now it should be safe.</li>"
                    "<li>Auto scaling features for the bar and graph visualization was not symmetric, so "
                    "it was difficult to read where the 0 lien was. Now it is symmetric, the 0 line is "
                    "thus always centered</li>"
                    "<li>When visualizing several neurons, the bars colors were cycled, and when no "
                    "more colors were found, it would switch to alpha channel, which made it "
                    "difficult to read. Now, all neurons have the same color</li>"
                    "<li>Papyrus segfaulted when a new script was created or opened and the ROS "
                    "master was down. This is now fixed.</li>"
                    "<li>It was not possible to save the scripts which were invalid. This was "
                    "problematic when you had to save your work and go, and did not have the time to"
                    " fix the problem. Now there's still a warning message, but saving is still"
                    " performed.</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.4.1</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Papyrus used to segfault when naming or renaming a script with a space in "
                    "the name, this was due to ROS not accepting spaces in topic names</li>"
                    "<li>The multiple XML argument for inputs is now optional and will default to "
                    "false when not specified. It is now encouraged not to add multiple = \"false\" "
                    "to make XML files more lightweight.</li>"
                    "<li>Papyrus did not check if an input was multiple or not before creating Link."
                    " Now it does, so it's not possible to connect several functions to an input "
                    "that is marked multiple</li>"
                    "<li>There was a problem saving the secondary argument: it used to be that Links"
                    " were saved as secondary if and only if it was a link from the same box it "
                    "started. Now it checks the real value.</li>"
                    "<li>When saving a script for the first time, it will use the script's name as "
                    "the default value for the XML file. So that the user doesn't have to manually "
                    "type it. Note: the script name is sanitized, so it means it is safe to use "
                    "spaces in script name: Visual Docking will be transformed as VisualDocking.xml."
                    " So please use spaces if you want to.</li>"
                    "<li>When opening a script, the state of the interface buttons (play, pause, "
                    "stop) was not initiated, thus was actually random. Now they are initiated to "
                    "the state where the script had not been launched (thus \"play\" visible only).</li>"
                    "<li>When a script was saved, there one warning message per constant function "
                    "box talking about missing description file. This is now fixed (built-in "
                    "Constants do not have description files)</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.4.0</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Implement a reliable, asynchronous communication with Kheops nodes.<br>"
                    "Now, starting, pausing and stopping a node is reliable (either when creating "
                    "and launching a node, or connecting to an existing node). The interface is "
                    "reliable and updated to the node's reported status.<br>"
                    "Basically, if you see a play button but no stop button, it means the script is "
                    "not launched. Is you see a play and a stop button, it means the script is "
                    "launched, but paused (clicking \"play\" will resume the execution).<br>"
                    "If you see a pause button, it means the script is launched and running "
                    "(clicking the pause button will pause the script's execution).>/li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.3.0</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Autosave feature is now implemented: when working on a script, it is "
                    "autosaved every 60 seconds (in a .autosave file). When one tries to open a "
                    "script, Papyrus will detect if there is an autosaved version of this file and "
                    "offer to open it instead (one can accept or refuse). Autosaved files are "
                    "destroyed when the script file is saved</li>"
                    "<li>When a script was opened, there was a bug where the script just before it "
                    "in the tab list would take its name. Thus we would have two tabs with the same "
                    "name. This is now fixed</li>"
                    "<li>The input and output slots of functions are now colorized according to "
                    "their types. At the moment, STRING-based slots are colorized cyan, SCALAR-based"
                    " types are left white and MATRIX-based slots are colorized pink. This makes it "
                    "much easier to see the compatible inputs</li>"
                    "<li>Function boxes can now have individual names. By default they don't have a "
                    "name and they function name is displayed. To give a name, edit the field in the"
                    " properties panel. When a function box has a name, it is displayed instead of "
                    "its function name. To remove a name (and restore displaying the function's "
                    "name), simply delete the name and validate.</li>"
                    "<li>Modifications in the properties panel can now be validated by pressing Enter "
                    "and cancelled by pressing ESCAPE (before this, one had to click on the Ok or "
                    "Cancel buttons ; now both choices are available).</li>"
                    "<li>Changes in the properties panel are now reflected on the scene as soon as "
                    "Enter is pressed (or Ok is clicked). Previous to this, it was necessary to "
                    "click on the scene with the mouse to trigger a repaint. This is no longer "
                    "necessary.</li>"
                    "<li>When changing some parameters in the properties panel, the script is now "
                    "set as \"modified\".</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.2.2</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Fix the icons for the ConstantBox that were going outside the box (this bug"
                    " happens when boxes were resized from 3 squares to 2). Now their icons are "
                    "displayed properly</li>"
                    "<li>I detected that if there was an issue (such as a segfault) during the save "
                    "operation, not only would the save operation fail, but the script file would be"
                    " emptied!<br>"
                    "This means it was possible to potentially lose a lot a work by trying to save.<br>"
                    "Now this is fixed: the saving is performed in a temporary file, and only when "
                    "this temporary file is written and successfully closed, it is copied on top of "
                    "the original script file.<br>"
                    "In case there's a segfault or some other error, only the modifications are lost"
                    " but the original script file is left untouched.</li>"
                    "</ul>"
                    "</li>"


                    "</ul>";

#endif // CHANGELOG_H
#ifndef CONNECTIVITYWINDOW_H
#define CONNECTIVITYWINDOW_H

#include <QTabWidget>

namespace Ui {
class ConnectivityWindow;
}

class ConnectivityWindow : public QTabWidget
{
    Q_OBJECT

public:
    explicit ConnectivityWindow(QSize inputSize, QSize outputSize, QWidget *parent = 0);
    ~ConnectivityWindow();

private:
    Ui::ConnectivityWindow *ui;
    QSize m_inputSize;   // Size of the input matrix
    QSize m_outputSize;  // Size of the output matrix
};

#endif // CONNECTIVITYWINDOW_H
#ifndef CONSTANTDIAGRAMBOX_H
#define CONSTANTDIAGRAMBOX_H

#include "diagrambox.h"
#include "outputslot.h"
#include "inputslot.h"

#include <QGraphicsItem>
#include <QIcon>
#include <QUuid>
#include <QPainter>

class ConstantDiagramBox : public DiagramBox
{
	Q_OBJECT

public:
	explicit ConstantDiagramBox(const QString &name,
	                            const QIcon &icon,
	                            OutputSlot *outputSlot,
	                            const QUuid &uuid = 0,
	                            QGraphicsItem *parent = 0);

	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};

#endif // CONSTANTDIAGRAMBOX_H
#ifndef CONSTANTFUNCTION_H
#define CONSTANTFUNCTION_H

#include "function.h"
#include "types.h"

class ConstantFunction : public Function
{
public:
    ConstantFunction(const QString &name,
                     const QString &iconPath,
                     const QIcon &icon,
                     const OutputType outputType);
};

#endif // CONSTANTFUNCTION_H
#ifndef CONSTANTS
#define CONSTANTS

/**
 * A set of project-wide constants
 */

// Define the name of the application
#define APP_NAME "Papyrus"

// Define the scroll factor for the QGraphicsView
#define SCALE_FACTOR 1.2

// Define the prefix in which to search for ressources
// TEMPORARY: should be replaced by a global 'settings' file for Papyrus
#define RESOURCE_DIR "/home/nschoe/workspace/Qt/papyrus/usr/share/"
// #define RESOURCE_DIR "/usr/share/"

// Define the size of the icons in the left 'Library' pane (in px)
#define LIBRARY_ICON_SIZE 40

// Define the default name for a new script
#define NEW_SCRIPT_DEFAULT_NAME "Untitled"

// Define the margin added to the scene's rectangle when resizing it (in px)
#define SCENE_RECT_MARGIN 200

// Define the name of the root element XML tag in the function's description files
#define XML_ROOT_ELEM "description"

// Major version number
#define MAJOR_VERSION 0

// Minor version number
#define MINOR_VERSION 5

// Bugfix version number
#define BUGFIX_VERSION 1

// Define the maximum number of rows / columns a matrix can have (for the spinbox)
#define MAX_ROWS 100000
#define MAX_COLS 100000

// Define the minimum and maximum allowed weight (for the double spinbox)
#define MIN_WEIGHT -10000000
#define MAX_WEIGHT 10000000

// Define the number of decimals for the double spinbox for link's weight
#define LINKS_NB_DECIMALS 10

// Define the minimum and maximum time value (for the double spinbox)
#define MIN_TIME_VALUE 0 //0.001
#define MAX_TIME_VALUE 10000

// Define the z-value for the links (used to put them behind slots to prevent hiding the slots)
#define LINKS_Z_VALUE -1.0

// Define the z-value for the data visualization windows (above boxes and links)
#define DATA_Z_VALUE 10

// Define the z-value for the rectangular comments (under everything)
#define COMMENTS_Z_VALUE -10.0

// Define the z-value for the neural boxes
#define BOXES_Z_VALUE 5

// Define the time (in minutes) after which the user is notified about modified, unsaved scripts
#define TIME_WARN_MODIFIED 10

// Define the organisation name, and domain for use with QSettings
#define ORGA "INSTAR Robotics"
#define DOMAIN "instar-robotics.com"

// Define the period for the autosave script
#define AUTOSAVE_PERIOD 60000 // 1 minute

// Time (in ms) during which status bar messages are displayed
#define MSG_DURATION 3000

#endif // CONSTANTS

#ifndef DATAFETCHER_H
#define DATAFETCHER_H

#include "types.h"

#include <QThread>
#include <QString>
#include <QList>
#include <ros/ros.h>

//QT_CHARTS_USE_NAMESPACE

class DataFetcher : public QThread
{
	Q_OBJECT

public:
	explicit DataFetcher(const QString &topicName, QObject *parent = nullptr);

	bool shouldQuit() const;
	void setShouldQuit(bool shouldQuit);

	VisualizationType visType() const;
	virtual void setVisType(VisualizationType type) = 0;

protected:
	QString m_topicName;
	bool m_shouldQuit;
	VisualizationType m_visType;

signals:
	void newMatrix(QList<double> *values);
};

#endif // DATAFETCHER_H
#ifndef DATAVISUALIZATION_H
#define DATAVISUALIZATION_H

#include "datafetcher.h"
#include "rossession.h"

#include <QWidget>
#include <QMenuBar>
#include <QMenu>
#include <QGraphicsScene>
#include <QThread>
#include <QString>

//QT_CHARTS_USE_NAMESPACE

class DiagramBox;
//class QBarSet;

class DataVisualization : public QWidget
{
	Q_OBJECT

public:
	DataVisualization(QWidget *parent = nullptr,
	                  ROSSession *rosSession = nullptr,
	                  QGraphicsScene *scene = nullptr,
	                  DiagramBox *box = nullptr);
	~DataVisualization();

protected:
	ROSSession *m_rosSession;
	QGraphicsScene *m_scene;
	DiagramBox *m_box;
	QMenuBar *m_menuBar;
	QMenu *m_typeMenu;
	DataFetcher *m_dataFetcher;
};

#endif // DATAVISUALIZATION_H
#ifndef DIAGRAMBOX_H
#define DIAGRAMBOX_H

#include "types.h"
#include "slot.h"
#include "outputslot.h"
#include "inputslot.h"
#include "datavisualization.h"
#include "rossession.h"

#include <set>

#include <QGraphicsItem>
#include <QIcon>
#include <QUuid>
#include <QGraphicsSvgItem>
#include <QGraphicsProxyWidget>

class Script;

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
	                    const QIcon &icon,
	                    OutputSlot *outputSlot,
	                    std::vector<InputSlot *> inputSlots,
	                    const QUuid &uuid = 0,
	                    QGraphicsItem *parent = 0);
	~DiagramBox();

	QRectF boundingRect() const override;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	Script *getScript();
	bool checkIfBoxInvalid();
	void updateTooltip();

	void showDataVis(ROSSession *rosSession);
	void setOutputSlotPos();

	QString scriptName();

	QString name() const;
	void setName(const QString &name);

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	int type();

	QString descriptionFile() const;
	void setDescriptionFile(const QString &descriptionPath);

	QIcon icon() const;
	void setIcon(const QIcon &icon);

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

	QGraphicsSvgItem *sizeIcon() const;
	void setSizeIcon(QGraphicsSvgItem *sizeIcon);

	bool publish() const;
	void setPublish(bool publish);

	QString topic() const;
	void setTopic(const QString &topic);

	QString iconFilepath() const;
	void setIconFilepath(const QString &value);

	QString libname() const;
	void setLibname(const QString &libname);

	DataVisualization *dataVis() const;
	void setDataVis(DataVisualization *dataVis);

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

protected:
	QVariant itemChange(GraphicsItemChange change, const QVariant &value);

	QString m_name;    // Name of the function
	QString m_title;   // Title of the box (user-friendly & customizable name)

	qreal m_bWidth;  // Overall width of the function's box
	qreal m_bHeight; // Overall height of the function's box
	qreal m_tHeight; // Height of the space in which th function's name is written

	MatrixShape m_matrixShape; // Shape (vector, row vector or col vector) if matrix

private:
	QUuid m_uuid;      // Unique ID of the function's box (to identify links for instance)
	QString m_libname; // Name of the library this function belongs to (for kheops's linking)
	QIcon m_icon;      // Icon representing the function

	QString m_descriptionFile; // Path to its XML description file (to get the icon when saving)

	OutputSlot *m_outputSlot;  // The output slot for this function's box
	std::vector<InputSlot *> m_inputSlots; // The set of input slots for this function's box

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
	QGraphicsSvgItem *m_sizeIcon; // Contains the svg that hints the box's size

	DataVisualization *m_dataVis; // The data visualization window for this box
	QGraphicsProxyWidget *m_dataProxy; // The proxy for the data visualization window

	bool m_isInvalid; // Whether this box is invalid
	BoxInvalidReason m_invalidReason; // Why this box is invalid
	bool m_swapCandidate; // Set to true when the user is dropping another box on top of this one

	QPointF m_oldPos; // Start position when moved (to enable undo)

private slots:
	void onDataVisClosed();
signals:
	void boxSelected(DiagramBox *); // Fired when the box is clicked on (used to signal PropertiesPanel)
};

#endif // DIAGRAMBOX_H
#ifndef DIAGRAMCHART_H
#define DIAGRAMCHART_H

#include "diagrambox.h"
#include "types.h"

#include <QChart>
#include <QBarSeries>
#include <QHorizontalBarSeries>
#include <QBarSet>
#include <QValueAxis>
#include <QGraphicsSceneHoverEvent>

QT_CHARTS_USE_NAMESPACE

class DiagramChart : public QChart
{
	Q_OBJECT

public:
	explicit DiagramChart(DiagramBox *box,
	                      QGraphicsItem *parent = nullptr,
	                      Qt::WindowFlags wFlags = Qt::WindowFlags());

	void hoverMoveEvent(QGraphicsSceneHoverEvent *evt);
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);

private:
	DiagramBox *m_box;          // The associated DiagramBox
	int m_size;                 // The number of data points in this chart
	MatrixShape m_matrixShape;  // Used to know if we display horizontally or vertically

	QBarSet m_barSet;

	QBarSeries m_barSeries;
	QValueAxis m_yAxis;

	QHorizontalBarSeries m_horizontalBarSeries;
	QValueAxis m_xAxis;

	qreal m_barMin;
	qreal m_barMax;

	qreal m_width;
	qreal m_height;

	ResizeType m_resizeType; // Type of resizing operation we are currently performing

public slots:
	void updateBarValues(const QList<qreal> matrix&);
};

#endif // DIAGRAMCHART_H
#ifndef DIAGRAMSCENE_H
#define DIAGRAMSCENE_H

#include "script.h"
#include "outputslot.h"
#include "inputslot.h"
#include "diagrambox.h"
#include "zone.h"

#include <QGraphicsScene>
#include <QUuid>
#include <QUndoStack>

// Forward declaration because of recursive include
class Script;
class PapyrusWindow;

/**
 * @brief The DiagramScene class represents the canvas on which neural scripts are drawn.
 * This is where @DiagramBox es are drag-n-dropped. There is one DiagramScene per script.
 */
class DiagramScene : public QGraphicsScene
{
	Q_OBJECT

public:
	explicit DiagramScene(QObject *parent = 0);
	~DiagramScene();

	void addBox(DiagramBox *newBox, const QPointF &position);

	bool checkForInvalidLinks();
	bool checkForInvalidity();

	void updateSceneRect();

	bool shouldDrawGrid() const;
	void setShouldDrawGrid(bool shouldDrawGrid);

	void removeItem(QGraphicsItem *item);

	int gridSize() const;

	Script *script() const;
	void setScript(Script *script);

	bool leftBtnDown() const;

	QGraphicsLineItem *line() const;

	PapyrusWindow *mainWindow() const;

	bool displayLabels() const;
	void setDisplayLabels(bool displayLabels);

	bool rightBtnDown() const;
	void setRightBtnDown(bool rightBtnDown);

	QGraphicsRectItem *rect() const;
	void setRect(QGraphicsRectItem *rect);

	QUndoStack *undoStack() const;
	void setUndoStack(QUndoStack *undoStack);

public slots:
	void toggleDisplayGrid(bool shouldDraw);
	void onOkBtnClicked(bool);
	void onCancelBtnClicked(bool);
	void onDisplayVisuClicked(bool);

protected:
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *evt);
	void dragEnterEvent(QGraphicsSceneDragDropEvent *evt);
	void dragLeaveEvent(QGraphicsSceneDragDropEvent *evt);
	void dragMoveEvent(QGraphicsSceneDragDropEvent *evt);
	void dropEvent(QGraphicsSceneDragDropEvent *evt);
	void keyPressEvent(QKeyEvent *evt);
	void deleteItem(Link *link);
	void deleteItem(DiagramBox *box);
	void deleteItem(Zone *zone);

	void drawBackground(QPainter *painter, const QRectF &rect);

private:
	PapyrusWindow *m_mainWindow; // A pointer to the main Papyrus window widget
	bool m_leftBtnDown;
	bool middleBtnIsDown;
	bool m_rightBtnDown;
	bool m_shouldDrawGrid;   // Whether to draw the grid or not
	int m_gridSize;          // Size (in px) of the grid
	QGraphicsLineItem *m_line; // The current line being drawn while clicking
	QGraphicsRectItem *m_rect; // The current rectangular section being drawn while clicking
	OutputSlot *m_oSlot;     // The slot from which the line being drawn originates
	Script *m_script;        // The script to which this scene is associated
	bool m_displayLabels;    // Whether or not to display input slots's names
	bool m_prevDisplayLabels;// Remembers the value of 'displayLabel' when creating links (to restore afterward)
	QUndoStack *m_undoStack; // The stack to allow for undo / redo commands

private slots:
	void onSelectionChanged();

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);

};

#endif // DIAGRAMSCENE_H
#ifndef DIAGRAMVIEW_H
#define DIAGRAMVIEW_H

#include <QGraphicsView>

/**
 * @brief The DiagramView class is used to render a @DiagramScene (the @DiagramScene holds
 * the data: the items added, their positions, etc.) and the DiagramView displays it. This
 * separation by Qt allows several views to be attached to a single scene (we will use
 * something like this when we implement the minimap).
 */
class DiagramView : public QGraphicsView
{
	Q_OBJECT
public:
	explicit DiagramView(QWidget *parent = 0);
	DiagramView(QGraphicsScene *scene, QWidget *parent = 0);

signals:

protected:
	void wheelEvent(QWheelEvent *evt);

public slots:
};

#endif // DIAGRAMVIEW_H
#ifndef FUNCTION_H
#define FUNCTION_H

#include "outputslot.h"
#include "inputslot.h"

#include <vector>

#include <QString>
#include <QIcon>
#include <QTreeWidgetItem>

/**
 * @brief The Function class describes a neural function for the @Library and @LibraryPanel
 * and comes from parsing a valid XML description file. Basically, the @Category class
 * holds several @Function.
 * The @Function class is only meant to be stored inside the @Library, but the actual neural
 * function is created as a @DiagramBox from this lightweight @Function object when it is
 * dropped on the @DiagramScene.
 */

class Function : public QTreeWidgetItem
{
public:
	Function(const QString &path);

	void updateTooltip();

	QString name() const;
	void setName(const QString &name);

	std::vector<InputSlot *> inputs() const;

	void addInputSlot (InputSlot *slot);

	OutputSlot *output() const;
	void setOutput(OutputSlot *output);

	QString descriptionFile() const;

	bool constant() const;
	void setConstant(bool constant);

	QString libName() const;
	void setLibName(const QString &libName);

	QString iconFilepath() const;
	void setIconFilepath(const QString &value);

	QString description() const;
	void setDescription(const QString &description);

	MatrixShape matrixShape() const;
	void setMatrixShape(const MatrixShape &matrixShape);

protected:
	QString m_name;
	QString m_descriptionFile;
	//    QIcon m_icon;
	std::vector<InputSlot *> m_inputs;
	OutputSlot *m_output;
	bool m_constant;   // Indicate whether this represents a constant input
	QString m_libName; // The name of the lib it belongs to, used for kheops to know where to look
	QString m_iconFilepath; // Filepath for the SVG icon, used when dropping the box on scene
	QString m_description;  // Description of the function
	MatrixShape m_matrixShape;
};

#endif // FUNCTION_H
#ifndef HELPERS_H
#define HELPERS_H

#include "types.h"
#include "papyruswindow.h"
#include "slot.h"

#include <ros/ros.h>

#include <QApplication>
#include <QString>
#include <QGraphicsSvgItem>
#include <QList>
#include <QDebug>
#include <QColor>

/**
 * This file provides a set of helpers that are used throughout this project.
 */

PapyrusWindow *getMainWindow();

QString outputTypeToString(const OutputType &outputType);
OutputType stringToOutputType(const QString &str);

QString inputTypeToString(const InputType &inputType);
InputType stringToInputType(const QString &str);

QString timeUnitToString(const TimeUnit &unit);

bool canLink(const OutputType &from, const InputType &to);

bool shapesMatch(DiagramBox *from, InputSlot *to, LinkInvalidReason *reason = nullptr);

void informUserAndCrash(const QString &text, const QString &title = QObject::tr("Papyrus is about to crash!"));

void rescaleSvgItem(QGraphicsSvgItem *svg, const QSizeF &size, const QPointF &pos, bool center = true);

void updateSizeIcon(DiagramBox *box);

bool areLinked(OutputSlot *oSlot, InputSlot *iSlot);

bool isFull(InputSlot *iSlot);

bool fileExists(const std::string& filename);

QList<QString> getKheopsNodes();

QString snakeCaseToPretty(const QString &str);

QString connectivityToString(const Connectivity &conn);

Connectivity stringToConnectivity(const QString &str);

QString mkTopicName(const QString &scriptName, const QString &topicName);

QString sanitizeTopicName(const QString &name);

bool isTopicNameValid(const QString &name);

QString ensureSlashPrefix(const QString &name);

QColor getTypeColor(const InputType inputType);

QColor getTypeColor(const OutputType outputType);

QString mkFilenameFromScript(const QString &scriptName);

qreal qrealAbsMax(const qreal a, qreal b);

MatrixShape stringToMatrixShape(const QString &str);

QString matrixShapeToString(const MatrixShape shape);

#endif // HELPERS_H
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
#ifndef INPUTSLOT_H
#define INPUTSLOT_H

#include "slot.h"
#include "types.h"

#include <QString>
#include <set>
#include <QGraphicsSimpleTextItem>
#include <QUuid>

class Link;

/**
 * @brief The InputSlot class is attached to a @DiagramBox. This is represented as the input
 * little circle to a @DiagramBox. Its job is to keep track of all @Link s from other
 * @Diagrambox es.
 * There is one @InputSlot per <input> in the function's XML description file.
 */
class InputSlot : public Slot
{
	Q_OBJECT

public:
	explicit InputSlot();
	explicit InputSlot(const QString &name);
	~InputSlot();

	bool multiple() const;
	void setMultiple(bool allowMultiple);

	std::vector<Link *> inputs() const;

	void addInput(Link *input, bool ignoreFull = false);
	void removeInput(Link *input);

	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
	QRectF boundingRect() const override;

	void updateLinks();

	InputType inputType() const;
	void setInputType(const InputType &inputType);

	bool canLink() const;
	void setCanLink(bool canLink);

	QGraphicsSimpleTextItem *label() const;
	void setLabel(QGraphicsSimpleTextItem *label);

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	bool checkSize() const;
	void setCheckSize(bool checkSize);

	QString description() const;
	void setDescription(const QString &description);

	MatrixShape matrixShape() const;
	void setMatrixShape(const MatrixShape &matrixShape);

private:
	QUuid m_uuid;              // Unid ID for the slot (used for kheops)
	bool m_multiple;           // Whether this slot can receive several links
	std::vector<Link *> m_inputs; // The set of links connected to this slot
	InputType m_inputType;     // Indicate type and connectivity of this input
	bool m_canLink;            // Indicate if this input can be linked to the current output when creating a Link
	QGraphicsSimpleTextItem *m_label; // A label that contains this input's name
	bool m_checkSize;          // Whether or not this input must check that sizes are correct (on SCALAR_MATRIX)
	QString m_description;     // Description of the input
	MatrixShape m_matrixShape; // The shape of the matrix it can accept
signals:
	void slotFull(); // Fired when trying to add a second input to a slot that doesn't allow multiple
};

#endif // INPUTSLOT_H
#ifndef LIBRARY_H
#define LIBRARY_H

#include "category.h"

#include <vector>

/**
 * @brief The Library class holds the @Categorie s which in turn hold all the @Function s
 * that have a valid XML description file. These @Function are then used as building blocks for
 * creating neural programs (by converting them to @DiagramBox es).
 */

class Library
{
public:
    Library();

    void addCategory(Category *category);

    std::vector<Category *> categories() const;
    void setCategories(const std::vector<Category *> &categories);

private:
    std::vector<Category *> m_categories;
};

#endif // LIBRARY_H
#ifndef LIBRARYPANEL_H
#define LIBRARYPANEL_H

#include <QTreeWidget>

/**
 * @brief The LibraryPanel class is the widget that goes in the left toolbar. It will hold
 * @Function s through the @Library.
 */
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
#ifndef LINK_H
#define LINK_H

#include "script.h"
#include "types.h"

#include <QGraphicsItem>
#include <QUuid>
#include <QPainterPath>
#include <QGraphicsLineItem>
#include <QString>

/**
 * @brief The Link class represents a link between neural functions (more precisely between
 * @DiagramBox es) Even more precisely, a Link is between a @DiagramBox 's @OutputSlot and
 * another (or same) @DiagramBox 's @InputSlot.
 * Depending on its type, it can have a weight.
 */

class InputSlot;
class OutputSlot;

Q_DECLARE_METATYPE(Connectivity);

class Link : public QObject, public QGraphicsItem
{
	Q_OBJECT

public:
	explicit Link(OutputSlot *f, InputSlot *t, QGraphicsItem *parent = 0);

	QRectF boundingRect() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0);
	QPainterPath shape() const;
	bool isStringLink();
	void updateTooltip();

	void addLinesToScene();
	void removeLinesFromScene();

	void updateLines();

	bool checkIfInvalid();

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	OutputSlot *from() const;
	void setFrom(OutputSlot *from);

	InputSlot *to() const;
	void setTo(InputSlot *to);

	bool secondary() const;
	void setSecondary(bool secondary);

	qreal weight() const;
	void setWeight(const qreal &weight);

	bool isInvalid() const;
	void setIsInvalid(bool isInvalid);

	bool selfLoop() const;

	QString value() const;
	void setValue(const QString &value);

	Connectivity connectivity() const;
	void setConnectivity(const Connectivity &connectivity);

	LinkInvalidReason invalidReason() const;
	void setInvalidReason(const LinkInvalidReason &invalidReason);

private:
	bool checkIfSelfLoop();

	QUuid m_uuid;           // Unique identifier
	OutputSlot *m_from;     // The OutputSlot this link goes from
	InputSlot *m_to;        // The InputSlot this link goes to
	bool m_secondary;       // Tells whether a link is a secondary link
	bool m_selfLoop;        // Tells whether a link loop back to the same function

	QGraphicsLineItem m_line;          // Main line that represents the link
	QGraphicsLineItem m_rightSegment;  // Right segment (for secondary links)
	QGraphicsLineItem m_leftSegment;   // Left segment (for secondary links)

	qreal m_weight;            // The weight associated to this link
	QString m_value;           // The string value associated to this link (when between strings)

	bool m_isInvalid; // Tells that this link is currently not valid (error in type, in sizes, etc.)
	LinkInvalidReason m_invalidReason; // Tell why a link is invalid

	Connectivity m_connectivity; // Only viable for MATRIX_MATRIX
};

#endif // LINK_H
#ifndef MATRIXFETCHER_H
#define MATRIXFETCHER_H

#include "scalarvisualization.h"

#include <QString>

#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"

class ScalarVisualization;
class MatrixVisualization;

class MatrixFetcher : public DataFetcher
{
	Q_OBJECT
public:
	explicit MatrixFetcher(const QString &topicName, MatrixVisualization *matrixVisualization, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, MatrixVisualization *matrixVisualization, VisualizationType type, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, VisualizationType type, QObject *parent = nullptr);

	void setVisType(VisualizationType type) override;

protected:
	void run() override;
	void fetchMatrix(const std_msgs::Float64MultiArray::ConstPtr& mat);

private:
	ScalarVisualization *m_scalarVisualization;
	MatrixVisualization *m_matrixVisualization;
	QList<double> m_dataList;
};

#endif // MATRIXFETCHER_H
#ifndef MATRIXVISUALIZATION_H
#define MATRIXVISUALIZATION_H

#include "datavisualization.h"
#include "rossession.h"

#include <QImage>
#include <QLabel>
#include <QVBoxLayout>


class MatrixVisualization : public DataVisualization
{
	Q_OBJECT
public:
	explicit MatrixVisualization(QWidget *parent = nullptr,
	                             ROSSession *rosSession = nullptr,
	                             QGraphicsScene *scene = nullptr,
	                             DiagramBox *box = nullptr);

private:
	QLabel *m_thermalImageLabel;
	QImage m_thermalImage;
	QVBoxLayout *m_vLayout;

private slots:
	void switchToThermal();
	void switchToImage();
	void switchToLandscape();
	void updateThermal(QList<double> *values);
};

#endif // MATRIXVISUALIZATION_H
#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <QApplication>
#include <QString>

void coloredMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg);

#endif //MESSAGE_HANDLER_H
#ifndef MOVECOMMAN_H
#define MOVECOMMAN_H

#include "diagrambox.h"

#include <QUndoCommand>

/**
 * @brief The MoveCommand class represents moving a @DiagramBox on a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */

class MoveCommand : public QUndoCommand
{
public:
	MoveCommand(DiagramBox *box, const QPointF &oldPos, QUndoCommand *parent = nullptr);
	void undo() override;
	void redo() override;

private:
	DiagramBox *m_box;
	QPointF m_oldPos;
	QPointF m_newPos;
};

#endif // MOVECOMMAN_H
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
#ifndef OUTPUTSLOT_H
#define OUTPUTSLOT_H

#include "slot.h"
#include "types.h"

#include <QString>
#include <set>

/**
 * @brief The specialized version of @Slot used for output.
 * They contain @Link s that goes out of a @DiagramBox.
 * Outputs slots react to the mouse by growing sightly when the mouse comes nears them, in order
 * to facilitate the creation of a link.
 */

class Link;

class OutputSlot : public Slot
{
    Q_OBJECT
public:
    explicit OutputSlot();

    std::set<Link *> outputs() const;

    void addOutput(Link *output);
    void removeOutput(Link *output);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QRectF boundingRect() const override;

    void mousePressEvent(QGraphicsSceneMouseEvent *evt);
    void hoverEnterEvent(QGraphicsSceneHoverEvent *evt);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *evt);

    bool isDrawingLine() const;
    void setIsDrawingLine(bool isDrawingLine);

    void updateLinks();

    OutputType outputType() const;
    void setOutputType(const OutputType &outputType);

private:
    std::set<Link *> m_outputs; // The set of links which leaves this slot
    bool m_isDrawingLine;       // Indicate if we are drawing an outgoing link
    OutputType m_outputType;    // Indicate whether this function (slot) outputs a matrix or scalar
};

#endif // OUTPUTSLOT_H
#ifndef PAPYRUSWINDOW_H
#define PAPYRUSWINDOW_H

#include "librarypanel.h"
#include "library.h"
#include "script.h"
#include "propertiespanel.h"
#include "rosnode.h"
#include "homepage.h"
#include "rossession.h"
#include "types.h"
#include "xmldescriptionreader.h"
#include "token.h"

#include <QMainWindow>
#include <QGraphicsScene>
#include <QDir>
#include <QSystemTrayIcon>
#include <QLineEdit>
#include <QLabel>
#include <QAction>
#include <QTimer>

namespace Ui {
class PapyrusWindow;
}

// Define the type of development environment (and where to go look for libraries)
enum DevelopmentType {
	RELEASE,
	DEBUG
};
Q_DECLARE_METATYPE(DevelopmentType) // This allows convertion from/to QVariant

// Define if we are asking the user for the DESCRIPTION or the LIBRARY path
enum PathType {
	PATH_DESC,
	PATH_LIB
};
Q_DECLARE_METATYPE(PathType);

/**
 * @brief The PapyrusWindow class is the main window of the application.
 * It contains the list of open @Script s, the @Library of @Function s,
 * the @DiagramView s, etc.
 */
class PapyrusWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit PapyrusWindow(int argc, char **argv, QWidget *parent = 0);
	~PapyrusWindow();

	void closeEvent(QCloseEvent *evt);

	void readSettings(QString &lastOpenedScripts, int *lastActiveScript);
	void writeSettings();
	Script *parseXmlScriptFile(const QString &scriptPath);
	void askForPath(bool displayWarning, const PathType &pathType);
	void parseOneLevel(QDir dir, XmlDescriptionReader *xmlReader);
	QString getDescriptionPath();
	QString getLibPath();
	void updateButtonsState();

	QDir description() const {return description_;}
	void setDescription(QDir description) {description_ = description;}

	Category *addTreeRoot(QString name);

	QLineEdit *librarySearchField() const;
	void setLibrarySearchField(QLineEdit *librarySearchField);

	Ui::PapyrusWindow *ui() const;

	Library *getLibrary() const;
	void setLibrary(Library *library);

	std::set<Script *> getScripts() const;
	void addScript(Script *script);

	Script *activeScript() const;

	PropertiesPanel *propertiesPanel() const;
	void setPropertiesPanel(PropertiesPanel *propertiesPanel);

	QSystemTrayIcon *getTrayIcon() const;

	RosNode *rosnode() const;
	void setRosnode(RosNode *rosnode);

	void spawnRosNode();

	HomePage *homePage() const;
	void setHomePage(HomePage *homePage);

	DevelopmentType developmentType() const;

	QString debugPath() const;

	QString releasePath() const;

	QString debugLibPath() const;

	QString releaseLibPath() const;

	QString keyFile() const;
	void setKeyFile(const QString &keyFile);

	QString ivFile() const;
	void setIvFile(const QString &ivFile);

	QString lastDir() const;
	void setLastDir(const QString &lastDir);

	QTimer *autoSaveTimer() const;
	void setAutoSaveTimer(QTimer *autoSaveTimer);

private:
	Ui::PapyrusWindow *m_ui;
	RosNode *m_rosnode;
	int m_argc;
	char **m_argv;
	QLabel *m_rosMasterStatus;
	LibraryPanel *libraryPanel_;
	QLineEdit *librarySearchField_;
	QString m_lastExpandedCategory;  // Name of the last category that was expanded before filtering
	int m_libraryParsingErrors;
	QDir description_;
	QSystemTrayIcon *trayIcon;
	Library *m_library;
	std::set<Script *> m_scripts;
	Script *m_activeScript;
	PropertiesPanel *m_propertiesPanel;
	HomePage *m_homePage;
	QLineEdit *m_runTimeDisplay;
	DevelopmentType m_developmentType;
	QAction *m_actionRelease;
	QAction *m_actionDebug;
	QString m_debugPath;        // Path where to search for description files in DEBUG mode
	QString m_releasePath;      // Path where to search for description files in RELEASE mode
	QString m_debugLibPath;     // Path where to search for library files in DEBUG mode
	QString m_releaseLibPath;   // Path where to search for library files in RELEASE mode
	QString m_keyFile;          // Path of the key file to crypt / decrypt scrip files
	QString m_ivFile;           // Path of the IV
	QString m_lastDir;          // Last directory visited for saving or loading files
	QTimer *m_autoSaveTimer;    // Timer to trigger auto save for scripts
	QString m_changelogVersion; // Used to know if we should show the changelog on launch
	QTimer *m_checkVersionTimer; // Timer that periodically check for new version release
	bool m_preventROSPopup;     // Prevents displaying ROS master pop-ups

signals:
	void toggleDisplayGrid(bool);
	void launched();

private slots:
	void filterLibraryNames(const QString &text);
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void onROSMasterChange(bool isOnline);
	void onScriptResumed();
	void onScriptPaused();
	void onScriptStopped();
	void updateStopWatch(int h, int m, int s, int ms);
	void updateDevelopmentEnvironment(QAction *action);
	void categoryExpanded(QTreeWidgetItem *item);
	void autoSave();
	void onPropPanelEnter();
	void onPropPanelEscape();
	void onLaunched();
	void openScript(QString path = "");
	void checkForNewRelease();
	void reEnableROSPopUp();

	void on_actionExit_triggered();

	void on_actionAntialiasing_toggled(bool antialiasing);

	void on_actionZoom_In_triggered();

	void on_actionZoom_Out_triggered();

	void on_actionZoom_Fit_triggered();

	void on_actionNew_script_hovered();

	void on_actionOpen_Script_hovered();

	void on_actionSave_Script_hovered();

	void on_actionZoom_In_hovered();

	void on_actionZoom_Out_hovered();

	void on_actionZoom_Fit_hovered();

	void on_actionNew_script_triggered();

	void on_actionDisplay_Grid_hovered();

	void on_actionDisplay_Grid_toggled(bool arg1);

	void on_actionAbout_Papyrus_triggered();

	void on_actionSave_Script_triggered();

	void on_actionOpen_Script_triggered();
	void on_tabWidget_currentChanged(int index);
	void on_tabWidget_tabBarDoubleClicked(int index);
	void on_actionClose_Script_triggered();
	void on_actionConnect_triggered();
	void on_actionRun_triggered();
	void on_actionStop_triggered();
	void on_actionScope_triggered();
	void on_actionEdit_paths_triggered();
	void on_actionShow_all_outputs_triggered();
	void on_actionHide_all_outputs_triggered();
	void on_actionList_shortcuts_triggered();
	void on_actionChangelog_triggered(bool isNewRelease = false);
	void on_actionReopen_last_scripts_triggered();
	void on_actionUndo_triggered();
	void on_actionRedo_triggered();
};

#endif // PAPYRUSWINDOW_H
#ifndef PROPDOUBLESPINBOX_H
#define PROPDOUBLESPINBOX_H

#include <QDoubleSpinBox>

/**
 * @brief The PropDoubleSpinBox class is the standard QDoubleSpinBox with only one minor
 * modification: its sizeHint has been made editable
 */

class PropDoubleSpinBox : public QDoubleSpinBox
{
public:
	PropDoubleSpinBox(QWidget *parent = nullptr);

	QSize sizeHint() const override;
	void setSizeHint(const QSize &sizeHint);

private:
	QSize m_sizeHint;
};

#endif // PROPDOUBLESPINBOX_H
#ifndef PROPERTIESPANEL_H
#define PROPERTIESPANEL_H

#include "diagrambox.h"
#include "link.h"
#include "script.h"
#include "zone.h"
#include "setcolorbutton.h"
#include "proplineedit.h"
#include "propdoublespinbox.h"

#include <QGroupBox>
#include <QLineEdit>
#include <QFrame>
#include <QLabel>
#include <QSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QFormLayout>

/**
 * @brief The PropertiesPanel class is used to display (and modify) the properties of selected
 * objects (such as slots, boxes, links, etc.)
 */

class PropertiesPanel : public QGroupBox
{
	Q_OBJECT

public:
	explicit PropertiesPanel(QWidget *parent = 0);

	void hideAllFrames(bool buttonsToo = false);
	void updateBoxProperties(DiagramBox *box);
	void updateLinkProperties(Link *link);
	void updateScriptProperties(Script *script);
	void updateZoneProperties(Zone *zone);

	void keyPressEvent(QKeyEvent *event);

	QFrame *boxFrame() const;
	void setBoxFrame(QFrame *boxFrame);

	QFrame *linkFrame() const;
	void setLinkFrame(QFrame *linkFrame);

	QLabel *boxName() const;
	void setBoxName(QLabel *boxName);

	QSpinBox *rowsInput() const;

	QSpinBox *colsInput() const;

	QPushButton *okBtn() const;
	void setOkBtn(QPushButton *okBtn);

	QPushButton *cancelBtn() const;
	void setCancelBtn(QPushButton *cancelBtn);

	QCheckBox *saveActivity() const;
	void setSaveActivity(QCheckBox *saveActivity);

	QDoubleSpinBox *linkWeight() const;

	QLabel *scriptName() const;
	void setScriptName(QLabel *scriptName);

	PropDoubleSpinBox *timeValue() const;
	void setTimeValue(PropDoubleSpinBox *timeValue);

	QComboBox *timeUnit() const;
	void setTimeUnit(QComboBox *timeUnit);

	QLineEdit *linkValue() const;
	void setLinkValue(QLineEdit *linkValue);

	QPushButton *displayVisu() const;
	void setDisplayVisu(QPushButton *displayVisu);

	PropLineEdit *zoneTitle() const;
	void setZoneTitle(PropLineEdit *zoneTitle);

	SetColorButton *zoneColor() const;
	void setZoneColor(SetColorButton *zoneColor);

	PropLineEdit *boxTitle() const;
	void setBoxTitle(PropLineEdit *boxTitle);

	QLabel *boxMatrixShape() const;
	void setBoxMatrixShape(QLabel *boxMatrixShape);

	QCheckBox *publish() const;
	void setPublish(QCheckBox *publish);

	PropLineEdit *topic() const;
	void setTopic(PropLineEdit *topic);

	QCheckBox *linkSecondary() const;
	void setLinkSecondary(QCheckBox *linkSecondary);

	QCheckBox *encrypt() const;
	void setEncrypt(QCheckBox *encrypt);

private:
	QVBoxLayout *m_panelLayout;  // The properties panel's main layout
	QFrame *m_scriptFrame;       // Container for script's properties
	QLabel *m_scriptName;        // Label used to change the script (and tab) name
	QLabel *m_timeLabel;         // Contains either "frequency" or "period"
	PropDoubleSpinBox *m_timeValue; // Used to input the script's frequency (or period)
	QComboBox *m_timeUnit;       // Used to select the unit (in Hz or ms)
	QCheckBox *m_encrypt;        // Whether or not the file is encrypted on save

	QFormLayout *m_boxLayout;  // Layout for the box properties (access needed to hide rows)
	QFrame *m_boxFrame;        // Container for box's properties
	QLabel *m_boxName;         // Display the name of the box
	PropLineEdit *m_boxTitle;     // Allow to see or change the box's custom name
	QLabel *m_boxOutputType;   // Display the box's output type (scalar, matrix)
	QLabel *m_boxMatrixShape;  // Display the shape of the function (when matrix)
	QSpinBox *m_rowsInput;     // Spin box to input number of rows in the output (if matrix)
	QSpinBox *m_colsInput;     // Spin box to input number of columns in the output (if matrix)
	QCheckBox *m_saveActivity; // To enable saving the activity of the box
	QCheckBox *m_publish;      // To enable publish the output of the function
	PropLineEdit *m_topic;        // To input the topic name for publishing
	QPushButton *m_displayVisu; // (TEMP) display the box's data vizualisation

	QFormLayout *m_linkLayout;    // Layout for the link properties (access needed to hide rows)
	QFrame *m_linkFrame;          // Container for link's properties
	QLabel *m_linkType;           // Display the type of the link
	QCheckBox *m_linkSecondary;   // Will display if the link is secondary or not
	QDoubleSpinBox *m_linkWeight; // Spin box to set the weight of the link
	QLineEdit *m_linkValue;       // Text field to enter the link's value (for string links)

	QFormLayout *m_zoneLayout;     // Contains the layout to display comment zone's properties
	QFrame *m_zoneFrame;           // Container for zone's properties
	PropLineEdit *m_zoneTitle;        // The comment zone's title
	SetColorButton *m_zoneColor;     // Holds the color of the comment zone

	// Will contain the sizes of connected matrix when the link is SPARSE_MATRIX
	QSize m_inputSize;
	QSize m_outputSize;

	QPushButton *m_okBtn;      // Button used to validate changes in parameters
	QPushButton *m_cancelBtn;  // Button used to discard changes in parameters and restore current

public slots:
	void displayBoxProperties(DiagramBox *box);
	void displayLinkProperties(Link *link);
	void displayScriptProperties(Script *script);
	void displayZoneProperties(Zone *zone);
	void convertTimeValues(int idx);
	void toggleTopic(bool isChecked);

private slots:
	void onTopicChanged(const QString &topic);

signals:
	void enterPressed();
	void escapePressed();
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
};

#endif // PROPERTIESPANEL_H
#ifndef PROPLINEEDIT_H
#define PROPLINEEDIT_H

#include <QLineEdit>

/**
 * @brief The PropLineEdit class is a standard @QLineEdit with only one minor modification: its
 * sizeHint can be changed
 */

class PropLineEdit : public QLineEdit
{
public:
	PropLineEdit(QWidget *parent = nullptr);
	PropLineEdit(const QString &contents, QWidget *parent = nullptr);

	QSize sizeHint() const override;
	void setSizeHint(const QSize &sizeHint);

private:
	QSize m_sizeHint; // Modified sizeHint
};

#endif // PROPLINEEDIT_H
#ifndef ROSNODE_H
#define ROSNODE_H

#include <QThread>

#include <ros/ros.h>

/**
 * @brief The RosNode class is used to handle ROS
 * operations
 */
class RosNode : public QThread
{
	Q_OBJECT
public:
	RosNode(int argc, char **argv);
	virtual ~RosNode();

	void init();
	void run() override;

	bool shouldQuit() const;
	void setShouldQuit(bool value);

private:
	int m_argc;
	char **m_argv;
	ros::Subscriber m_sub;
	bool m_shouldQuit;       // Used by Papyrus to cleanly exit the thread

signals:
	void rosMasterChanged(bool isOnline);
};

#endif // ROSNODE_H
#ifndef ROSSESSION_H
#define ROSSESSION_H

#include <ros/ros.h>

//#include "script.h"
#include "types.h"

#include <QThread>
#include <QString>
#include <QSet>

#include "diagnostic_msgs/KeyValue.h"

/**
 * @brief The ROSSession class contains parameters related to the current ROS session (connection
 * with a Kheops node
 */
class ROSSession : public QThread
{
	Q_OBJECT

public:
	ROSSession(const QString &nodeName, QObject *parent = nullptr);
	~ROSSession();

	void addToHotList(QUuid uuid);

	bool shouldQuit() const;
	void setShouldQuit(bool shouldQuit);

	QString nodeName() const;
	void setNodeName(const QString &nodeName);

private:
	bool m_shouldQuit;       // Used to properly exit the thread
	QString m_nodeName;      // Name of the node it should listen
	QSet<QUuid> m_hotList;   // List of functions' uuids whose output to activate on-the-fly
	bool m_isFirstRun;       // To differentiate a 'resume' on first launch or after a pause

	void run() override;
	void handleStatusChange(const diagnostic_msgs::KeyValue::ConstPtr& msg);
	void activateOutput(QUuid uuid);

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void scriptResumed();
	void scriptPaused();
	void scriptStopped();
};

#endif // ROSSESSION_H
#ifndef SCALARFETCHER_H
#define SCALARFETCHER_H

#include "datafetcher.h"
#include "types.h"

#include <QDebug>
#include <QBarSet>
#include <QSplineSeries>

#include <ros/ros.h>
#include "std_msgs/Float64.h"

QT_CHARTS_USE_NAMESPACE

class ScalarVisualization;

class ScalarFetcher : public DataFetcher
{
	Q_OBJECT
public:
	explicit ScalarFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent = nullptr);
	explicit ScalarFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, VisualizationType type, QObject *parent = nullptr);

	void setVisType(VisualizationType type) override;

protected:
	void run() override;

	void fetchScalar(const std_msgs::Float64::ConstPtr& scalar);

private:
	ScalarVisualization *m_scalarVisualization;
};

#endif // SCALARFETCHER_H
#ifndef SCALARVISUALIZATION_H
#define SCALARVISUALIZATION_H

#include "types.h"
#include "datavisualization.h"
#include "rossession.h"

#include <QBarSet>
#include <QBarSeries>
#include <QHorizontalBarSeries>
#include <QChart>
#include <QChartView>
#include <QSplineSeries>
#include <QValueAxis>
#include <QVBoxLayout>
#include <QList>

#include <vector>

QT_CHARTS_USE_NAMESPACE

class ScalarVisualization : public DataVisualization
{
	Q_OBJECT
public:
	ScalarVisualization(QWidget *parent = nullptr,
	                    ROSSession *rosSession = nullptr,
	                    QGraphicsScene *scene = nullptr,
	                    DiagramBox *box = nullptr);
	void mousePressEvent(QMouseEvent *evt);

	void updateBarValues(const std::vector<qreal> &values);
	void pushGraphValues(const std::vector<qreal> &values);

protected:
	void createCharts();

	int m_size;
	int m_idx;

	QBarSet *m_barSet;
	QBarSeries *m_barSeries;
	QHorizontalBarSeries *m_horizontalBarSeries;
	QChart *m_barChart;
	QValueAxis *m_barAxisY;
	QValueAxis *m_barAxisX;
	QChartView *m_barView;
	qreal m_barMin;
	qreal m_barMax;
	MatrixShape m_matrixShape;  // used to determine wether to display a ROW or COL vector (when applicable)

	QList<QSplineSeries *> m_graphSeries;
	QChart *m_graphChart;
	QVBoxLayout *m_vLayout;
	QValueAxis *m_graphAxisX;
	QValueAxis *m_graphAxisY;
	QChartView *m_graphView;
	qreal m_graphMin;
	qreal m_graphMax;

protected slots:
	void switchToBar();
	void switchToGraph();
};

#endif // SCALARVISUALIZATION_H
#ifndef SCRIPT_H
#define SCRIPT_H

#include "diagramscene.h"
#include "types.h"
#include "rossession.h"

#include <QString>
#include <QFile>
#include <QDir>
#include <QUuid>
#include <QTimer>

enum ScriptStatus {
	INVALID_SCRIPT_STATUS,
	SCRIPT_RUNNING,
	SCRIPT_PAUSED
};

// Forward declaration because of recursive includes
class DiagramScene;

Q_DECLARE_METATYPE(TimeUnit) // This allows convertion from/to QVariant

/**
 * @brief The Script class represents a neural scripts. An XML file is associated to it,
 * as well as the @QGraphicsScene that contains its functions boxes
 */

class Script : public QObject
{
	Q_OBJECT
public:
	Script(DiagramScene *scene, const QString &name = "");
	~Script();

	void save(const QString &basePath = QDir::homePath(),
	          bool isAutoSave = false);

	void updateTextStyle();
	void runOrPause();
	void run();
	void pause();
	void stop();
	ScriptStatus queryScriptStatus();
	void setupROSSession();

	QString name() const;
	void setName(const QString &name);

	QString filePath() const;
	void setFilePath(const QString &filePath);

	DiagramScene *scene() const;

	bool modified() const;

	void setStatusModified(bool isModified);

	bool isInvalid() const;
	void setIsInvalid(bool isInvalid);

	double timeValue() const;
	void setTimeValue(double timeValue);

	TimeUnit timeUnit() const;
	void setTimeUnit(const TimeUnit &timeUnit);

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	bool encrypt() const;
	void setEncrypt(bool encrypt);

	ROSSession *rosSession() const;
	void setRosSession(ROSSession *rosSession);

	bool isActiveScript() const;
	void setIsActiveScript(bool isActiveScript);

	bool hasTab() const;
	void setHasTab(bool hasTab);

	bool isRunning() const;
	void setIsRunning(bool isRunning);

	bool isPaused() const;
	void setIsPaused(bool isPaused);

	QString nodeName() const;
	void setNodeName(const QString &nodeName);

public slots:
	void warnAboutModifiedScript();

private:
	DiagramScene *m_scene; // The associated scene for this script
	bool m_hasTab; // Tells whether the scripts has a tab in the tabwidget or not
	ROSSession *m_rosSession; // The associated ROS Session for this script
	QString m_name;        // Pretty name of the script (to display in tabs for instance)
	QString m_nodeName;    // The ROS node name of the script
	QString m_filePath;    // Path of the (XML) file in which to save this script
	bool m_modified;       // Whether there was some changes since last save
	bool m_isInvalid;      // Whether this script is currently invalid (and thus prevent saving)
	double m_timeValue;    // The RT Token time (either frequency or period)
	TimeUnit m_timeUnit;   // Whether the time value is a frequency or a period
	QUuid m_uuid;          // UUID for the RT Token (needed by kheops)
	QTimer *m_modifiedNotifTimer; // Timer to display a system tray notification when unsaved for more than X minutes
	bool m_encrypt;        // Whether the XML script should be encrypted on save (to protect IP)
	std::string m_key;     // AES Key used to encrypt the file
	std::string m_iv;      // AES IV used to encrypt the file
	bool m_isActiveScript; // Tells this script if it's the currently active one
	bool m_isRunning;      // Tells whether this script is running (launched)
	bool m_isPaused;       // Tells whether this script is paused while running

private slots:
	void onROSSessionMessage(const QString &msg, MessageUrgency urgency = MSG_INFO);
	void onScriptResumed();
	void onScriptPaused();
	void onScriptStopped();
	void onTimeElapsed(int h, int m, int s, int ms);
//	void temporaryCheckLaunch();

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void scriptResumed();
	void scriptPaused();
	void scriptStopped();
	void timeElapsed(int h, int m, int s, int ms);
};

#endif // SCRIPT_H
#ifndef SETCOLORBUTTON_H
#define SETCOLORBUTTON_H

#include <QPushButton>
#include <QColor>

class SetColorButton : public QPushButton
{
	Q_OBJECT

public:
	explicit SetColorButton(QWidget *parent = nullptr);

	void updateColor();

	QColor color() const;
	void setColor(const QColor &color);

private:
	QColor m_color;

private slots:
	void changeColor();
};

#endif // SETCOLORBUTTON_H
#ifndef SLOT_H
#define SLOT_H

#include <QObject>
#include <QString>
#include <QGraphicsItem>

//#include <diagrambox.h>
class DiagramBox;

/**
 * @brief The Slot class defines an argument slot, i.e. an item will either receive
 * a connection from another box's output slot, or from which a connection leaves to
 * reach another box' input slot.
 * This class is meant to be subclassed (see @InputSlot and @OutputSlot).
 */

/*
enum InputType {
    SCALAR_SCALAR,
    SCALAR_MATRIX,
    MATRIX_MATRIX,
    SPARSE_MATRIX
};

enum OutputType {
    SCALAR,
    MATRIX
};
*/

class Slot : public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    explicit Slot(QGraphicsItem *parent = 0);
    explicit Slot(QString &name, QGraphicsItem *parent = 0);
    ~Slot();

    QString name() const;
    void setName(const QString &name);

    virtual QRectF boundingRect() const = 0;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) = 0;

    qreal dist() const;
    void setDist(const qreal &dist);

    DiagramBox *box() const;
    void setBox(DiagramBox *box);

protected:
    QString m_name;    // The name of this slot
    qreal m_dist;      // Distance to the mouse (used to highlight the slot when mouse approach)
    DiagramBox *m_box; // The DiagramBox that is associated with this Slot
};

#endif // SLOT_H
#ifndef SWAPBOXESCOMMAND_H
#define SWAPBOXESCOMMAND_H

#include "diagrambox.h"
#include "diagramscene.h"
#include "link.h"

#include <QUndoCommand>

/**
 * @brief The SwapBoxesCommand class represents swapping two functions. This is used to provide
 * Undo/Redo functionality.
 */

class SwapBoxesCommand : public QUndoCommand
{
public:
	SwapBoxesCommand(DiagramScene *scene, DiagramBox *toSwap, DiagramBox *newBox,
	                 QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	DiagramScene *m_scene;  // The scene in which the swap takes place
	DiagramBox *m_toSwap;   // The box to be swapped
	DiagramBox *m_newBox;   // The box to add

	QList<Link *> m_outputLinks; // List of all links that are FROM m_toSwap
	QHash<QString, QList<Link *>> m_inputLinks; // List of all links that are TO m_toSwap, sorted by input slots
};

#endif // SWAPBOXESCOMMAND_H
#ifndef TOKEN_H
#define TOKEN_H

// Define token and password to check Papyrus version on gitlab
#define GITLAB_TOKEN "gitlab+deploy-token-1"
#define GITLAB_PWD "xz7LJDr-bMe6wZYHzLze"

#endif // TOKEN_H
#ifndef TYPES_H
#define TYPES_H

/**
 * This is just a collection of types (no functions!) in order to simply the include process
 */

// Define the type that an @InputSlot accepts
enum InputType{
	INVALID_INPUT_TYPE,
	SCALAR_SCALAR,
	SCALAR_MATRIX,
	MATRIX_MATRIX,
	STRING_INPUT
};

// Define the type that an @OutputSlot (hence a function, really) outputs
enum OutputType{
	INVALID_OUTPUT_TYPE,
	SCALAR,
	MATRIX,
	STRING
};

// Define the shape of a matrix-typed @DiagramBox
enum MatrixShape {
	INVALID_MATRIX_SHAPE,
	SHAPE_NONE,
	POINT,
	VECT,
	ROW_VECT,
	COL_VECT
};

// Define whether the time value is a frequency in Hz or a period in ms for a @Script
enum TimeUnit{
	INVALID_UNIT,
	HZ,
	MS
};

// Define the urgency of a message that will be shown on the status bar (and then its color)
enum MessageUrgency {
	INVALID_MESSAGE_URGENCY,
	MSG_INFO,
	MSG_WARNING,
	MSG_ERROR
};

// Define the type of connectivity for MATRIX_MATRIX links
enum Connectivity {
	INVALID_CONNECTIVITY,
	ONE_TO_ONE,
	ONE_TO_ALL,
	ONE_TO_NEI
};

// Define the type of visualization for data
enum VisualizationType {
	INVALID_VISUALIZATION_TYPE,
	BAR,        // scalar and vector
	GRAPH,      // scalar and vector
	IMAGE,      // matrix
	GRAYSCALE,  // matrix
	LANDSCAPE   // matrix
};

// Define the different reasons why a Link can be invalid
enum LinkInvalidReason {
	INVALID_INVALID_REASON = 0, // MUST be zero (because it uses bitwise OR)
	TYPES_INCOMPATIBLE     = 0b1,
	SIZES_DONT_MATCH       = 0b10,
	SHAPE_MUST_BE_POINT    = 0b100,
	SHAPE_MUST_BE_VECT     = 0b1000,
	SHAPE_MUST_BE_ROW_VECT = 0b10000,
	SHAPE_MUST_BE_COL_VECT = 0b100000
};

// Defining bitwise operations for LinkInvalidReason because in C++11, enums are scoped
inline LinkInvalidReason operator|(LinkInvalidReason a, LinkInvalidReason b) {
	return static_cast<LinkInvalidReason>(static_cast<int>(a) | static_cast<int>(b));
}

inline LinkInvalidReason operator&(LinkInvalidReason a, LinkInvalidReason b) {
	return static_cast<LinkInvalidReason>(static_cast<int>(a) & static_cast<int>(b));
}

// Define the different reasons why a box can be invalid
enum BoxInvalidReason {
	INVALID_BOX_INVALID_REASON = 0, // MUST be zero (because it uses bitwise OR)
	INPUT_FULL                 = 0b1,
	BOX_MUST_BE_POINT          = 0b10,
	BOX_MUST_BE_VECT           = 0b100,
	BOX_MUST_BE_ROW_VECT       = 0b1000,
	BOX_MUST_BE_COL_VECT       = 0b10000,
};

// Defining bitwise operations for BoxInvalidReason because in C++11, enums are scoped
inline BoxInvalidReason operator|(BoxInvalidReason a, BoxInvalidReason b) {
	return static_cast<BoxInvalidReason>(static_cast<int>(a) | static_cast<int>(b));
}

inline BoxInvalidReason operator&(BoxInvalidReason a, BoxInvalidReason b) {
	return static_cast<BoxInvalidReason>(static_cast<int>(a) & static_cast<int>(b));
}

#endif // TYPES_H
#ifndef UPDATEBOXCOMMAND_H
#define UPDATEBOXCOMMAND_H

#include "propertiespanel.h"
#include "diagrambox.h"

#include <QUndoCommand>

/**
 * @brief The UpdateZoneCommand class represents updating a @DiagramBox's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateBoxCommand : public QUndoCommand
{
public:
	UpdateBoxCommand(PropertiesPanel *panel, DiagramBox *box, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	DiagramBox *m_box;        // The box which we are modifying parameters

	// Old parameters (ones the box had before updating its parameters)
	QString m_oldTitle;
	int m_oldRows;
	int m_oldCols;
	bool m_oldActivity;
	bool m_oldPublish;
	QString m_oldTopic;

	// New parameters (ones the box will have after updating its parameters)
	QString m_newTitle;
	int m_newRows;
	int m_newCols;
	bool m_newActivity;
	bool m_newPublish;
	QString m_newTopic;
};

#endif // UPDATEBOXCOMMAND_H
#ifndef UPDATELINKCOMMAND_H
#define UPDATELINKCOMMAND_H

#include "propertiespanel.h"
#include "link.h"

#include <QUndoCommand>

/**
 * @brief The UpdateLinkCommand class represents updating a @Link's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateLinkCommand : public QUndoCommand
{
public:
	UpdateLinkCommand(PropertiesPanel *panel, Link *link, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	Link *m_link;             // The link which we are modifying parameters

	// Old parameters
	qreal m_oldWeight;
	QString m_oldValue;
	bool m_oldSecondary;

	// New parameters
	qreal m_newWeight;
	QString m_newValue;
	bool m_newSecondary;
};

#endif // UPDATELINKCOMMAND_H
#ifndef UPDATESCRIPTCOMMAND_H
#define UPDATESCRIPTCOMMAND_H

#include "propertiespanel.h"
#include "script.h"

#include <QUndoCommand>

/**
 * @brief The UpdateScriptCommand class represents updating a @Script's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateScriptCommand : public QUndoCommand
{
public:
        UpdateScriptCommand(PropertiesPanel *panel, Script *script, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	Script *m_script;           // The script which we are modifying parameters

	// Old parameters
	qreal m_oldTime;
	TimeUnit m_oldUnit;
	bool m_oldEncrypt;

	// New parameters
	qreal m_newTime;
	TimeUnit m_newUnit;
	bool m_newEncrypt;
};

#endif // UPDATESCRIPTCOMMAND_H
#ifndef UPDATEZONECOMMAND_H
#define UPDATEZONECOMMAND_H

#include "propertiespanel.h"
#include "zone.h"

#include <QUndoCommand>

/**
 * @brief The UpdateZoneCommand class represents updating a @Zone's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateZoneCommand : public QUndoCommand
{
public:
	UpdateZoneCommand(PropertiesPanel *panel, Zone *zone, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	Zone *m_zone;             // The zone which we are modifying parameters

	// Old parameters
	QString m_oldTitle;
	QColor m_oldColor;

	// New parameters
	QString m_newTitle;
	QColor m_newColor;
};

#endif // UPDATEZONECOMMAND_H
#ifndef XMLDESCRIPTIONREADER_H
#define XMLDESCRIPTIONREADER_H

#include "category.h"
#include "slot.h"

#include <QXmlStreamReader>

/**
 * @brief The XmlDescriptionReader class is a XML file parser whose task is to populate a @Library
 * object with the @Categorie  and descriptions of neural @Function s, read from their XML description
 * file.
 */

class XmlDescriptionReader
{
public:
	XmlDescriptionReader(Category *category );

	bool read(QIODevice *device, const QString &descriptionFile);
private:
//    Library *m_library;
	Category *m_category;
	QXmlStreamReader reader;

	void readDescription(const QString &descriptionFile);
	void readAllFunctions(const QString &libName, const QString &descriptionFile);
	void readOneFunction(const QString &libName, const QString &descriptionFile);
	void readName(Function *function);
	void readInputs(Function *function);
	QString readIcon();
	void readParameterName(Slot *paramSlot);
	void readParameterType(OutputSlot *paramSlot);
	void readParameterType(InputSlot *paramSlot);
	void readParameterDesc(InputSlot *paramSlot);
	void readOutput(Function *function);
	void readFunctionDesc(Function *function);
};

#endif // XMLDESCRIPTIONREADER_H
#ifndef XMLSCRIPTREADER_H
#define XMLSCRIPTREADER_H

#include "script.h"
#include "diagrambox.h"
#include "outputslot.h"
#include "inputslot.h"
#include "library.h"

#include <QXmlStreamReader>
#include <QUuid>

/**
 * @brief The XmlScriptReader class is an XML parser that reads an XML @Script file
 * to populate the @GraphicsScene.
 */

class XmlScriptReader
{
public:
	explicit XmlScriptReader(Script *script, const QString &descriptionPath, Library *library);
	bool read(QIODevice *device);

	QString errorString() const;

	QPointF centerView() const;
	void setCenterView(const QPointF &centerView);

	Library *library() const;
	void setLibrary(Library *library);

private:
	QXmlStreamReader reader;
	QString m_errorString;
	Script *m_script;
	QString m_descriptionPath;
	QPointF m_centerView;
	Library *m_library;

	void readScript();
	void readFunction(std::map<QUuid, DiagramBox *> *allBoxes,
	                  std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readFunctionName(QString &name);
	void readFunctionTitle(QString &title);
	void readFunctionSave(bool *save);
	void readPublishTopic(QString &topic, bool *publish);
	void readInputSlots(std::vector<InputSlot *> *inputSlots,
	                    std::map<QUuid, DiagramBox *> *allBoxes,
	                    std::set<std::pair<QUuid, Link *>> *incompleteLinks);
	void readOutputSlot(OutputSlot *outputSlot, int *rows, int *cols);
	void readUUID(QUuid *uuid);
	void readPosition(QPointF *pos);
	void readLinks(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
	               std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readLink(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
	              std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readZone();
};

#endif // XMLSCRIPTREADER_H
#ifndef ZONE_H
#define ZONE_H

#include <QGraphicsObject>
#include <QGraphicsItem>
#include <QColor>
#include <QGraphicsItemGroup>

enum ResizeType {
	NO_RESIZE,
	RESIZE_TOP,
	RESIZE_RIGHT,
	RESIZE_BOTTOM,
	RESIZE_LEFT
};

class Zone : public QGraphicsObject
{
public:
	explicit Zone(QGraphicsObject *parent = nullptr);
	explicit Zone(qreal x, qreal y, qreal w, qreal h, QGraphicsObject *parent = nullptr);
	~Zone();

	QRectF boundingRect() const override;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	QVariant itemChange(GraphicsItemChange change, const QVariant &value);
	void mousePressEvent(QGraphicsSceneMouseEvent *event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	void hoverMoveEvent(QGraphicsSceneHoverEvent *event);

	void updateGroup(bool startFromScratch = false);
	void updateLinks();

	// Getters / Setters

	qreal width() const;
	void setWidth(const qreal &width);

	qreal height() const;
	void setHeight(const qreal &height);

	QColor color() const;
	void setColor(const QColor &color);

	QString title() const;
	void setTitle(const QString &title);

private:
	qreal m_width;
	qreal m_height;
	QColor m_color;
	QString m_title; // The title of the comment zone (should be kept small)
	ResizeType m_resizeType;
};

#endif // ZONE_H
#ifndef ADDBOXCOMMAND_H
#define ADDBOXCOMMAND_H

#include "diagrambox.h"
#include "diagramscene.h"

#include <QUndoCommand>

/**
 * @brief The AddBoxCommand class represents adding a @DiagramBox to aa @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddBoxCommand : public QUndoCommand
{
public:
	AddBoxCommand(DiagramScene *scene, DiagramBox *box, const QPointF &initialPos, QUndoCommand *parent = nullptr);
	~AddBoxCommand();

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene; // The scene in which the box should be added
	DiagramBox *m_box;     // The DiagramBox to add to the scene
	QPointF m_initialPos;  // The initial position at which the box is added
};

#endif // ADDBOXCOMMAND_H
#ifndef ADDLINKCOMMAND_H
#define ADDLINKCOMMAND_H

#include "diagramscene.h"
#include "link.h"
#include "inputslot.h"
#include "outputslot.h"

#include <QUndoCommand>

/**
 * @brief The AddLinkCommand class represents adding a @Link to a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddLinkCommand : public QUndoCommand
{
public:
	AddLinkCommand(DiagramScene *scene, Link *link, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene; // The scene in which the link is added
	Link *m_link;          // The link to add to the scene
	OutputSlot *m_from;    // The output slot the link is from
	InputSlot *m_to;       // The input slot the link goes to
	bool m_isFirst;        // Flag that indicates wheather a redo() action is the first one
};

#endif // ADDLINKCOMMAND_H
#ifndef ADDZONECOMMAND_H
#define ADDZONECOMMAND_H

#include "diagramscene.h"
#include "zone.h"

#include <QUndoCommand>

/**
 * @brief The AddZoneCommand class represents adding a @Zone to a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddZoneCommand : public QUndoCommand
{
public:
	AddZoneCommand(DiagramScene *scene, Zone *zone, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene;  // The scene in which to add the zone
	Zone *m_zone;           // The zone to add to the scene
};

#endif // ADDZONECOMMAND_H
#ifndef CATEGORY_H
#define CATEGORY_H

#include "function.h"

#include <vector>
#include <QTreeWidgetItem>

/**
 * @brief The Category class holds a number of neural @Function 's descriptions that are part of the
 * same category.
 * This is used with the @Library and the @LibraryPanel. The idea is to group similar functions
 * by themes. The categories are simply made by parsing the directory name in which the function's
 * descriptions are saved.
 */

class Category : public QTreeWidgetItem
{
public:
    Category(const QString &name);

    QString name() const;
    void setName(const QString &name);

private:
    QString m_name;
};

#endif // CATEGORY_H
#ifndef CHANGELOG_H
#define CHANGELOG_H

#include <QString>

QString changelog = "<h3>CHANGELOG</h3>"
                    "<ul>"

                    "<li><strong>v0.5.1</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>When saving the same script, without adding any boxes, and just making "
                    "position changes, or changing the weight of a link, the order in which "
                    "functions and links were written in the XML was not deterministic. It has been "
                    "fixed, and now both functions and links are written in order of their UUID. "
                    "Moving boxes or changing link weights will produce only a small git diff for "
                    "the coordinates / weights.</li>"
                    "<li>"
                    "<li>There was a segfault when closing Papyrus after opening two scripts. This "
                    "prevented the normal cleaning procedures to happen and thus settings to be "
                    "saved (among other things). This is now fixed.</li>"
                    "<li>The libraries in the library panel on the left are not sorted "
                    "alphabetically (they were sorted in the reverse order)</li>"
                    "<li>Lots of XML information about the function boxes were saved in the script's"
                    " XML and thus any changed in alexandria (such as the icon, the name of inputs, "
                    "etc.) were not being reloaded. Now only the minimum information is stored in "
                    "the XML, instead, much of the information regarding a box are being re-parsed "
                    "from the library when a script is opened.</li>"
                    "<li>There was a compilation error on older Qt version, because a QStringRef "
                    "could not be used in the same manner, this was converted back to QString to fix"
                    " this.</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.5.0</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Parse <description> tags from XML description files and display a "
                    "function's description (and its input) when hovering the library</li>"
                    "<li>Fix the graphical glitch where a link seemed to \"go out in the distance\" "
                    "when a Comment Zone was resized and the box was now out of the zone</li>"
                    "<li>The full version MAJOR.MINOR.BUGFIX is now displayed in the Home Page and "
                    "the About dialog (the bugfix version was missing) which allows the user to know"
                    " which version he is running</li>"
                    "<li>Only one Comment Zone was parsed & loaded when a script was opened, this is"
                    " now fixed</li>"
                    "<li>Function boxes used to be transparent which made them weird-looking when "
                    "placed inside Comment Zones, now their background is white</li>"
                    "<li><strong>(new feature)</strong> Papyrus now periodically checks the gitlab repository for a "
                    "new release and when it finds one, it warns the user with a dialog</li>"
                    "<li><strong>(new feature)</strong> Papyrus will now re-open with the last "
                    "opened scripts (and the last active script) by default. It is possible, however"
                    " to disable this feature in the options</li>"
                    "<li><strong>(new feature)</strong> The full CHANGELOG is now visible in the "
                    "Help > CHANGELOG menu. Also, when it's the first time a new version is "
                    "launched, the CHANGELOG is automatically displayed.</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.4.2</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Fix the big issue when events on properties panel (such as clicking OK, CANCEL, "
                    "or pressing ENTER) was propagated to all open scenes. It was thus unsafe to edit "
                    "several scripts at the same time. Now it should be safe.</li>"
                    "<li>Auto scaling features for the bar and graph visualization was not symmetric, so "
                    "it was difficult to read where the 0 lien was. Now it is symmetric, the 0 line is "
                    "thus always centered</li>"
                    "<li>When visualizing several neurons, the bars colors were cycled, and when no "
                    "more colors were found, it would switch to alpha channel, which made it "
                    "difficult to read. Now, all neurons have the same color</li>"
                    "<li>Papyrus segfaulted when a new script was created or opened and the ROS "
                    "master was down. This is now fixed.</li>"
                    "<li>It was not possible to save the scripts which were invalid. This was "
                    "problematic when you had to save your work and go, and did not have the time to"
                    " fix the problem. Now there's still a warning message, but saving is still"
                    " performed.</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.4.1</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Papyrus used to segfault when naming or renaming a script with a space in "
                    "the name, this was due to ROS not accepting spaces in topic names</li>"
                    "<li>The multiple XML argument for inputs is now optional and will default to "
                    "false when not specified. It is now encouraged not to add multiple = \"false\" "
                    "to make XML files more lightweight.</li>"
                    "<li>Papyrus did not check if an input was multiple or not before creating Link."
                    " Now it does, so it's not possible to connect several functions to an input "
                    "that is marked multiple</li>"
                    "<li>There was a problem saving the secondary argument: it used to be that Links"
                    " were saved as secondary if and only if it was a link from the same box it "
                    "started. Now it checks the real value.</li>"
                    "<li>When saving a script for the first time, it will use the script's name as "
                    "the default value for the XML file. So that the user doesn't have to manually "
                    "type it. Note: the script name is sanitized, so it means it is safe to use "
                    "spaces in script name: Visual Docking will be transformed as VisualDocking.xml."
                    " So please use spaces if you want to.</li>"
                    "<li>When opening a script, the state of the interface buttons (play, pause, "
                    "stop) was not initiated, thus was actually random. Now they are initiated to "
                    "the state where the script had not been launched (thus \"play\" visible only).</li>"
                    "<li>When a script was saved, there one warning message per constant function "
                    "box talking about missing description file. This is now fixed (built-in "
                    "Constants do not have description files)</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.4.0</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Implement a reliable, asynchronous communication with Kheops nodes.<br>"
                    "Now, starting, pausing and stopping a node is reliable (either when creating "
                    "and launching a node, or connecting to an existing node). The interface is "
                    "reliable and updated to the node's reported status.<br>"
                    "Basically, if you see a play button but no stop button, it means the script is "
                    "not launched. Is you see a play and a stop button, it means the script is "
                    "launched, but paused (clicking \"play\" will resume the execution).<br>"
                    "If you see a pause button, it means the script is launched and running "
                    "(clicking the pause button will pause the script's execution).>/li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.3.0</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Autosave feature is now implemented: when working on a script, it is "
                    "autosaved every 60 seconds (in a .autosave file). When one tries to open a "
                    "script, Papyrus will detect if there is an autosaved version of this file and "
                    "offer to open it instead (one can accept or refuse). Autosaved files are "
                    "destroyed when the script file is saved</li>"
                    "<li>When a script was opened, there was a bug where the script just before it "
                    "in the tab list would take its name. Thus we would have two tabs with the same "
                    "name. This is now fixed</li>"
                    "<li>The input and output slots of functions are now colorized according to "
                    "their types. At the moment, STRING-based slots are colorized cyan, SCALAR-based"
                    " types are left white and MATRIX-based slots are colorized pink. This makes it "
                    "much easier to see the compatible inputs</li>"
                    "<li>Function boxes can now have individual names. By default they don't have a "
                    "name and they function name is displayed. To give a name, edit the field in the"
                    " properties panel. When a function box has a name, it is displayed instead of "
                    "its function name. To remove a name (and restore displaying the function's "
                    "name), simply delete the name and validate.</li>"
                    "<li>Modifications in the properties panel can now be validated by pressing Enter "
                    "and cancelled by pressing ESCAPE (before this, one had to click on the Ok or "
                    "Cancel buttons ; now both choices are available).</li>"
                    "<li>Changes in the properties panel are now reflected on the scene as soon as "
                    "Enter is pressed (or Ok is clicked). Previous to this, it was necessary to "
                    "click on the scene with the mouse to trigger a repaint. This is no longer "
                    "necessary.</li>"
                    "<li>When changing some parameters in the properties panel, the script is now "
                    "set as \"modified\".</li>"
                    "</ul>"
                    "</li>"

                    "<li><strong>v0.2.2</strong></li>"
                    "<li>"
                    "<ul>"
                    "<li>Fix the icons for the ConstantBox that were going outside the box (this bug"
                    " happens when boxes were resized from 3 squares to 2). Now their icons are "
                    "displayed properly</li>"
                    "<li>I detected that if there was an issue (such as a segfault) during the save "
                    "operation, not only would the save operation fail, but the script file would be"
                    " emptied!<br>"
                    "This means it was possible to potentially lose a lot a work by trying to save.<br>"
                    "Now this is fixed: the saving is performed in a temporary file, and only when "
                    "this temporary file is written and successfully closed, it is copied on top of "
                    "the original script file.<br>"
                    "In case there's a segfault or some other error, only the modifications are lost"
                    " but the original script file is left untouched.</li>"
                    "</ul>"
                    "</li>"


                    "</ul>";

#endif // CHANGELOG_H
#ifndef CONNECTIVITYWINDOW_H
#define CONNECTIVITYWINDOW_H

#include <QTabWidget>

namespace Ui {
class ConnectivityWindow;
}

class ConnectivityWindow : public QTabWidget
{
    Q_OBJECT

public:
    explicit ConnectivityWindow(QSize inputSize, QSize outputSize, QWidget *parent = 0);
    ~ConnectivityWindow();

private:
    Ui::ConnectivityWindow *ui;
    QSize m_inputSize;   // Size of the input matrix
    QSize m_outputSize;  // Size of the output matrix
};

#endif // CONNECTIVITYWINDOW_H
#ifndef CONSTANTDIAGRAMBOX_H
#define CONSTANTDIAGRAMBOX_H

#include "diagrambox.h"
#include "outputslot.h"
#include "inputslot.h"

#include <QGraphicsItem>
#include <QIcon>
#include <QUuid>
#include <QPainter>

class ConstantDiagramBox : public DiagramBox
{
	Q_OBJECT

public:
	explicit ConstantDiagramBox(const QString &name,
	                            const QIcon &icon,
	                            OutputSlot *outputSlot,
	                            const QUuid &uuid = 0,
	                            QGraphicsItem *parent = 0);

	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};

#endif // CONSTANTDIAGRAMBOX_H
#ifndef CONSTANTFUNCTION_H
#define CONSTANTFUNCTION_H

#include "function.h"
#include "types.h"

class ConstantFunction : public Function
{
public:
    ConstantFunction(const QString &name,
                     const QString &iconPath,
                     const QIcon &icon,
                     const OutputType outputType);
};

#endif // CONSTANTFUNCTION_H
#ifndef CONSTANTS
#define CONSTANTS

/**
 * A set of project-wide constants
 */

// Define the name of the application
#define APP_NAME "Papyrus"

// Define the scroll factor for the QGraphicsView
#define SCALE_FACTOR 1.2

// Define the prefix in which to search for ressources
// TEMPORARY: should be replaced by a global 'settings' file for Papyrus
#define RESOURCE_DIR "/home/nschoe/workspace/Qt/papyrus/usr/share/"
// #define RESOURCE_DIR "/usr/share/"

// Define the size of the icons in the left 'Library' pane (in px)
#define LIBRARY_ICON_SIZE 40

// Define the default name for a new script
#define NEW_SCRIPT_DEFAULT_NAME "Untitled"

// Define the margin added to the scene's rectangle when resizing it (in px)
#define SCENE_RECT_MARGIN 200

// Define the name of the root element XML tag in the function's description files
#define XML_ROOT_ELEM "description"

// Major version number
#define MAJOR_VERSION 0

// Minor version number
#define MINOR_VERSION 5

// Bugfix version number
#define BUGFIX_VERSION 1

// Define the maximum number of rows / columns a matrix can have (for the spinbox)
#define MAX_ROWS 100000
#define MAX_COLS 100000

// Define the minimum and maximum allowed weight (for the double spinbox)
#define MIN_WEIGHT -10000000
#define MAX_WEIGHT 10000000

// Define the number of decimals for the double spinbox for link's weight
#define LINKS_NB_DECIMALS 10

// Define the minimum and maximum time value (for the double spinbox)
#define MIN_TIME_VALUE 0 //0.001
#define MAX_TIME_VALUE 10000

// Define the z-value for the links (used to put them behind slots to prevent hiding the slots)
#define LINKS_Z_VALUE -1.0

// Define the z-value for the data visualization windows (above boxes and links)
#define DATA_Z_VALUE 10

// Define the z-value for the rectangular comments (under everything)
#define COMMENTS_Z_VALUE -10.0

// Define the z-value for the neural boxes
#define BOXES_Z_VALUE 5

// Define the time (in minutes) after which the user is notified about modified, unsaved scripts
#define TIME_WARN_MODIFIED 10

// Define the organisation name, and domain for use with QSettings
#define ORGA "INSTAR Robotics"
#define DOMAIN "instar-robotics.com"

// Define the period for the autosave script
#define AUTOSAVE_PERIOD 60000 // 1 minute

// Time (in ms) during which status bar messages are displayed
#define MSG_DURATION 3000

#endif // CONSTANTS

#ifndef DATAFETCHER_H
#define DATAFETCHER_H

#include "types.h"

#include <QThread>
#include <QString>
#include <QList>
#include <ros/ros.h>

//QT_CHARTS_USE_NAMESPACE

class DataFetcher : public QThread
{
	Q_OBJECT

public:
	explicit DataFetcher(const QString &topicName, QObject *parent = nullptr);

	bool shouldQuit() const;
	void setShouldQuit(bool shouldQuit);

	VisualizationType visType() const;
	virtual void setVisType(VisualizationType type) = 0;

protected:
	QString m_topicName;
	bool m_shouldQuit;
	VisualizationType m_visType;

signals:
	void newMatrix(QList<double> *values);
};

#endif // DATAFETCHER_H
#ifndef DATAVISUALIZATION_H
#define DATAVISUALIZATION_H

#include "datafetcher.h"
#include "rossession.h"

#include <QWidget>
#include <QMenuBar>
#include <QMenu>
#include <QGraphicsScene>
#include <QThread>
#include <QString>

//QT_CHARTS_USE_NAMESPACE

class DiagramBox;
//class QBarSet;

class DataVisualization : public QWidget
{
	Q_OBJECT

public:
	DataVisualization(QWidget *parent = nullptr,
	                  ROSSession *rosSession = nullptr,
	                  QGraphicsScene *scene = nullptr,
	                  DiagramBox *box = nullptr);
	~DataVisualization();

protected:
	ROSSession *m_rosSession;
	QGraphicsScene *m_scene;
	DiagramBox *m_box;
	QMenuBar *m_menuBar;
	QMenu *m_typeMenu;
	DataFetcher *m_dataFetcher;
};

#endif // DATAVISUALIZATION_H
#ifndef DIAGRAMBOX_H
#define DIAGRAMBOX_H

#include "types.h"
#include "slot.h"
#include "outputslot.h"
#include "inputslot.h"
#include "datavisualization.h"
#include "rossession.h"

#include <set>

#include <QGraphicsItem>
#include <QIcon>
#include <QUuid>
#include <QGraphicsSvgItem>
#include <QGraphicsProxyWidget>

class Script;

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
	                    const QIcon &icon,
	                    OutputSlot *outputSlot,
	                    std::vector<InputSlot *> inputSlots,
	                    const QUuid &uuid = 0,
	                    QGraphicsItem *parent = 0);
	~DiagramBox();

	QRectF boundingRect() const override;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	Script *getScript();
	bool checkIfBoxInvalid();
	void updateTooltip();

	void showDataVis(ROSSession *rosSession);
	void setOutputSlotPos();

	QString scriptName();

	QString name() const;
	void setName(const QString &name);

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	int type();

	QString descriptionFile() const;
	void setDescriptionFile(const QString &descriptionPath);

	QIcon icon() const;
	void setIcon(const QIcon &icon);

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

	QGraphicsSvgItem *sizeIcon() const;
	void setSizeIcon(QGraphicsSvgItem *sizeIcon);

	bool publish() const;
	void setPublish(bool publish);

	QString topic() const;
	void setTopic(const QString &topic);

	QString iconFilepath() const;
	void setIconFilepath(const QString &value);

	QString libname() const;
	void setLibname(const QString &libname);

	DataVisualization *dataVis() const;
	void setDataVis(DataVisualization *dataVis);

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

protected:
	QVariant itemChange(GraphicsItemChange change, const QVariant &value);

	QString m_name;    // Name of the function
	QString m_title;   // Title of the box (user-friendly & customizable name)

	qreal m_bWidth;  // Overall width of the function's box
	qreal m_bHeight; // Overall height of the function's box
	qreal m_tHeight; // Height of the space in which th function's name is written

	MatrixShape m_matrixShape; // Shape (vector, row vector or col vector) if matrix

private:
	QUuid m_uuid;      // Unique ID of the function's box (to identify links for instance)
	QString m_libname; // Name of the library this function belongs to (for kheops's linking)
	QIcon m_icon;      // Icon representing the function

	QString m_descriptionFile; // Path to its XML description file (to get the icon when saving)

	OutputSlot *m_outputSlot;  // The output slot for this function's box
	std::vector<InputSlot *> m_inputSlots; // The set of input slots for this function's box

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
	QGraphicsSvgItem *m_sizeIcon; // Contains the svg that hints the box's size

	DataVisualization *m_dataVis; // The data visualization window for this box
	QGraphicsProxyWidget *m_dataProxy; // The proxy for the data visualization window

	bool m_isInvalid; // Whether this box is invalid
	BoxInvalidReason m_invalidReason; // Why this box is invalid
	bool m_swapCandidate; // Set to true when the user is dropping another box on top of this one

	QPointF m_oldPos; // Start position when moved (to enable undo)

private slots:
	void onDataVisClosed();
signals:
	void boxSelected(DiagramBox *); // Fired when the box is clicked on (used to signal PropertiesPanel)
};

#endif // DIAGRAMBOX_H
#ifndef DIAGRAMCHART_H
#define DIAGRAMCHART_H

#include "diagrambox.h"
#include "types.h"

#include <QChart>
#include <QBarSeries>
#include <QHorizontalBarSeries>
#include <QBarSet>
#include <QValueAxis>
#include <QGraphicsSceneHoverEvent>

QT_CHARTS_USE_NAMESPACE

class DiagramChart : public QChart
{
	Q_OBJECT

public:
	explicit DiagramChart(DiagramBox *box,
	                      QGraphicsItem *parent = nullptr,
	                      Qt::WindowFlags wFlags = Qt::WindowFlags());

	void hoverMoveEvent(QGraphicsSceneHoverEvent *evt);
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);

private:
	DiagramBox *m_box;          // The associated DiagramBox
	int m_size;                 // The number of data points in this chart
	MatrixShape m_matrixShape;  // Used to know if we display horizontally or vertically

	QBarSet m_barSet;

	QBarSeries m_barSeries;
	QValueAxis m_yAxis;

	QHorizontalBarSeries m_horizontalBarSeries;
	QValueAxis m_xAxis;

	qreal m_barMin;
	qreal m_barMax;

	qreal m_width;
	qreal m_height;

	ResizeType m_resizeType; // Type of resizing operation we are currently performing

public slots:
	void updateBarValues(const QList<qreal> matrix&);
};

#endif // DIAGRAMCHART_H
#ifndef DIAGRAMSCENE_H
#define DIAGRAMSCENE_H

#include "script.h"
#include "outputslot.h"
#include "inputslot.h"
#include "diagrambox.h"
#include "zone.h"

#include <QGraphicsScene>
#include <QUuid>
#include <QUndoStack>

// Forward declaration because of recursive include
class Script;
class PapyrusWindow;

/**
 * @brief The DiagramScene class represents the canvas on which neural scripts are drawn.
 * This is where @DiagramBox es are drag-n-dropped. There is one DiagramScene per script.
 */
class DiagramScene : public QGraphicsScene
{
	Q_OBJECT

public:
	explicit DiagramScene(QObject *parent = 0);
	~DiagramScene();

	void addBox(DiagramBox *newBox, const QPointF &position);

	bool checkForInvalidLinks();
	bool checkForInvalidity();

	void updateSceneRect();

	bool shouldDrawGrid() const;
	void setShouldDrawGrid(bool shouldDrawGrid);

	void removeItem(QGraphicsItem *item);

	int gridSize() const;

	Script *script() const;
	void setScript(Script *script);

	bool leftBtnDown() const;

	QGraphicsLineItem *line() const;

	PapyrusWindow *mainWindow() const;

	bool displayLabels() const;
	void setDisplayLabels(bool displayLabels);

	bool rightBtnDown() const;
	void setRightBtnDown(bool rightBtnDown);

	QGraphicsRectItem *rect() const;
	void setRect(QGraphicsRectItem *rect);

	QUndoStack *undoStack() const;
	void setUndoStack(QUndoStack *undoStack);

public slots:
	void toggleDisplayGrid(bool shouldDraw);
	void onOkBtnClicked(bool);
	void onCancelBtnClicked(bool);
	void onDisplayVisuClicked(bool);

protected:
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *evt);
	void dragEnterEvent(QGraphicsSceneDragDropEvent *evt);
	void dragLeaveEvent(QGraphicsSceneDragDropEvent *evt);
	void dragMoveEvent(QGraphicsSceneDragDropEvent *evt);
	void dropEvent(QGraphicsSceneDragDropEvent *evt);
	void keyPressEvent(QKeyEvent *evt);
	void deleteItem(Link *link);
	void deleteItem(DiagramBox *box);
	void deleteItem(Zone *zone);

	void drawBackground(QPainter *painter, const QRectF &rect);

private:
	PapyrusWindow *m_mainWindow; // A pointer to the main Papyrus window widget
	bool m_leftBtnDown;
	bool middleBtnIsDown;
	bool m_rightBtnDown;
	bool m_shouldDrawGrid;   // Whether to draw the grid or not
	int m_gridSize;          // Size (in px) of the grid
	QGraphicsLineItem *m_line; // The current line being drawn while clicking
	QGraphicsRectItem *m_rect; // The current rectangular section being drawn while clicking
	OutputSlot *m_oSlot;     // The slot from which the line being drawn originates
	Script *m_script;        // The script to which this scene is associated
	bool m_displayLabels;    // Whether or not to display input slots's names
	bool m_prevDisplayLabels;// Remembers the value of 'displayLabel' when creating links (to restore afterward)
	QUndoStack *m_undoStack; // The stack to allow for undo / redo commands

private slots:
	void onSelectionChanged();

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);

};

#endif // DIAGRAMSCENE_H
#ifndef DIAGRAMVIEW_H
#define DIAGRAMVIEW_H

#include <QGraphicsView>

/**
 * @brief The DiagramView class is used to render a @DiagramScene (the @DiagramScene holds
 * the data: the items added, their positions, etc.) and the DiagramView displays it. This
 * separation by Qt allows several views to be attached to a single scene (we will use
 * something like this when we implement the minimap).
 */
class DiagramView : public QGraphicsView
{
	Q_OBJECT
public:
	explicit DiagramView(QWidget *parent = 0);
	DiagramView(QGraphicsScene *scene, QWidget *parent = 0);

signals:

protected:
	void wheelEvent(QWheelEvent *evt);

public slots:
};

#endif // DIAGRAMVIEW_H
#ifndef FUNCTION_H
#define FUNCTION_H

#include "outputslot.h"
#include "inputslot.h"

#include <vector>

#include <QString>
#include <QIcon>
#include <QTreeWidgetItem>

/**
 * @brief The Function class describes a neural function for the @Library and @LibraryPanel
 * and comes from parsing a valid XML description file. Basically, the @Category class
 * holds several @Function.
 * The @Function class is only meant to be stored inside the @Library, but the actual neural
 * function is created as a @DiagramBox from this lightweight @Function object when it is
 * dropped on the @DiagramScene.
 */

class Function : public QTreeWidgetItem
{
public:
	Function(const QString &path);

	void updateTooltip();

	QString name() const;
	void setName(const QString &name);

	std::vector<InputSlot *> inputs() const;

	void addInputSlot (InputSlot *slot);

	OutputSlot *output() const;
	void setOutput(OutputSlot *output);

	QString descriptionFile() const;

	bool constant() const;
	void setConstant(bool constant);

	QString libName() const;
	void setLibName(const QString &libName);

	QString iconFilepath() const;
	void setIconFilepath(const QString &value);

	QString description() const;
	void setDescription(const QString &description);

	MatrixShape matrixShape() const;
	void setMatrixShape(const MatrixShape &matrixShape);

protected:
	QString m_name;
	QString m_descriptionFile;
	//    QIcon m_icon;
	std::vector<InputSlot *> m_inputs;
	OutputSlot *m_output;
	bool m_constant;   // Indicate whether this represents a constant input
	QString m_libName; // The name of the lib it belongs to, used for kheops to know where to look
	QString m_iconFilepath; // Filepath for the SVG icon, used when dropping the box on scene
	QString m_description;  // Description of the function
	MatrixShape m_matrixShape;
};

#endif // FUNCTION_H
#ifndef HELPERS_H
#define HELPERS_H

#include "types.h"
#include "papyruswindow.h"
#include "slot.h"

#include <ros/ros.h>

#include <QApplication>
#include <QString>
#include <QGraphicsSvgItem>
#include <QList>
#include <QDebug>
#include <QColor>

/**
 * This file provides a set of helpers that are used throughout this project.
 */

PapyrusWindow *getMainWindow();

QString outputTypeToString(const OutputType &outputType);
OutputType stringToOutputType(const QString &str);

QString inputTypeToString(const InputType &inputType);
InputType stringToInputType(const QString &str);

QString timeUnitToString(const TimeUnit &unit);

bool canLink(const OutputType &from, const InputType &to);

bool shapesMatch(DiagramBox *from, InputSlot *to, LinkInvalidReason *reason = nullptr);

void informUserAndCrash(const QString &text, const QString &title = QObject::tr("Papyrus is about to crash!"));

void rescaleSvgItem(QGraphicsSvgItem *svg, const QSizeF &size, const QPointF &pos, bool center = true);

void updateSizeIcon(DiagramBox *box);

bool areLinked(OutputSlot *oSlot, InputSlot *iSlot);

bool isFull(InputSlot *iSlot);

bool fileExists(const std::string& filename);

QList<QString> getKheopsNodes();

QString snakeCaseToPretty(const QString &str);

QString connectivityToString(const Connectivity &conn);

Connectivity stringToConnectivity(const QString &str);

QString mkTopicName(const QString &scriptName, const QString &topicName);

QString sanitizeTopicName(const QString &name);

bool isTopicNameValid(const QString &name);

QString ensureSlashPrefix(const QString &name);

QColor getTypeColor(const InputType inputType);

QColor getTypeColor(const OutputType outputType);

QString mkFilenameFromScript(const QString &scriptName);

qreal qrealAbsMax(const qreal a, qreal b);

MatrixShape stringToMatrixShape(const QString &str);

QString matrixShapeToString(const MatrixShape shape);

#endif // HELPERS_H
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
#ifndef INPUTSLOT_H
#define INPUTSLOT_H

#include "slot.h"
#include "types.h"

#include <QString>
#include <set>
#include <QGraphicsSimpleTextItem>
#include <QUuid>

class Link;

/**
 * @brief The InputSlot class is attached to a @DiagramBox. This is represented as the input
 * little circle to a @DiagramBox. Its job is to keep track of all @Link s from other
 * @Diagrambox es.
 * There is one @InputSlot per <input> in the function's XML description file.
 */
class InputSlot : public Slot
{
	Q_OBJECT

public:
	explicit InputSlot();
	explicit InputSlot(const QString &name);
	~InputSlot();

	bool multiple() const;
	void setMultiple(bool allowMultiple);

	std::vector<Link *> inputs() const;

	void addInput(Link *input, bool ignoreFull = false);
	void removeInput(Link *input);

	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
	QRectF boundingRect() const override;

	void updateLinks();

	InputType inputType() const;
	void setInputType(const InputType &inputType);

	bool canLink() const;
	void setCanLink(bool canLink);

	QGraphicsSimpleTextItem *label() const;
	void setLabel(QGraphicsSimpleTextItem *label);

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	bool checkSize() const;
	void setCheckSize(bool checkSize);

	QString description() const;
	void setDescription(const QString &description);

	MatrixShape matrixShape() const;
	void setMatrixShape(const MatrixShape &matrixShape);

private:
	QUuid m_uuid;              // Unid ID for the slot (used for kheops)
	bool m_multiple;           // Whether this slot can receive several links
	std::vector<Link *> m_inputs; // The set of links connected to this slot
	InputType m_inputType;     // Indicate type and connectivity of this input
	bool m_canLink;            // Indicate if this input can be linked to the current output when creating a Link
	QGraphicsSimpleTextItem *m_label; // A label that contains this input's name
	bool m_checkSize;          // Whether or not this input must check that sizes are correct (on SCALAR_MATRIX)
	QString m_description;     // Description of the input
	MatrixShape m_matrixShape; // The shape of the matrix it can accept
signals:
	void slotFull(); // Fired when trying to add a second input to a slot that doesn't allow multiple
};

#endif // INPUTSLOT_H
#ifndef LIBRARY_H
#define LIBRARY_H

#include "category.h"

#include <vector>

/**
 * @brief The Library class holds the @Categorie s which in turn hold all the @Function s
 * that have a valid XML description file. These @Function are then used as building blocks for
 * creating neural programs (by converting them to @DiagramBox es).
 */

class Library
{
public:
    Library();

    void addCategory(Category *category);

    std::vector<Category *> categories() const;
    void setCategories(const std::vector<Category *> &categories);

private:
    std::vector<Category *> m_categories;
};

#endif // LIBRARY_H
#ifndef LIBRARYPANEL_H
#define LIBRARYPANEL_H

#include <QTreeWidget>

/**
 * @brief The LibraryPanel class is the widget that goes in the left toolbar. It will hold
 * @Function s through the @Library.
 */
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
#ifndef LINK_H
#define LINK_H

#include "script.h"
#include "types.h"

#include <QGraphicsItem>
#include <QUuid>
#include <QPainterPath>
#include <QGraphicsLineItem>
#include <QString>

/**
 * @brief The Link class represents a link between neural functions (more precisely between
 * @DiagramBox es) Even more precisely, a Link is between a @DiagramBox 's @OutputSlot and
 * another (or same) @DiagramBox 's @InputSlot.
 * Depending on its type, it can have a weight.
 */

class InputSlot;
class OutputSlot;

Q_DECLARE_METATYPE(Connectivity);

class Link : public QObject, public QGraphicsItem
{
	Q_OBJECT

public:
	explicit Link(OutputSlot *f, InputSlot *t, QGraphicsItem *parent = 0);

	QRectF boundingRect() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0);
	QPainterPath shape() const;
	bool isStringLink();
	void updateTooltip();

	void addLinesToScene();
	void removeLinesFromScene();

	void updateLines();

	bool checkIfInvalid();

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	OutputSlot *from() const;
	void setFrom(OutputSlot *from);

	InputSlot *to() const;
	void setTo(InputSlot *to);

	bool secondary() const;
	void setSecondary(bool secondary);

	qreal weight() const;
	void setWeight(const qreal &weight);

	bool isInvalid() const;
	void setIsInvalid(bool isInvalid);

	bool selfLoop() const;

	QString value() const;
	void setValue(const QString &value);

	Connectivity connectivity() const;
	void setConnectivity(const Connectivity &connectivity);

	LinkInvalidReason invalidReason() const;
	void setInvalidReason(const LinkInvalidReason &invalidReason);

private:
	bool checkIfSelfLoop();

	QUuid m_uuid;           // Unique identifier
	OutputSlot *m_from;     // The OutputSlot this link goes from
	InputSlot *m_to;        // The InputSlot this link goes to
	bool m_secondary;       // Tells whether a link is a secondary link
	bool m_selfLoop;        // Tells whether a link loop back to the same function

	QGraphicsLineItem m_line;          // Main line that represents the link
	QGraphicsLineItem m_rightSegment;  // Right segment (for secondary links)
	QGraphicsLineItem m_leftSegment;   // Left segment (for secondary links)

	qreal m_weight;            // The weight associated to this link
	QString m_value;           // The string value associated to this link (when between strings)

	bool m_isInvalid; // Tells that this link is currently not valid (error in type, in sizes, etc.)
	LinkInvalidReason m_invalidReason; // Tell why a link is invalid

	Connectivity m_connectivity; // Only viable for MATRIX_MATRIX
};

#endif // LINK_H
#ifndef MATRIXFETCHER_H
#define MATRIXFETCHER_H

#include "scalarvisualization.h"

#include <QString>

#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"

class ScalarVisualization;
class MatrixVisualization;

class MatrixFetcher : public DataFetcher
{
	Q_OBJECT
public:
	explicit MatrixFetcher(const QString &topicName, MatrixVisualization *matrixVisualization, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, MatrixVisualization *matrixVisualization, VisualizationType type, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, VisualizationType type, QObject *parent = nullptr);

	void setVisType(VisualizationType type) override;

protected:
	void run() override;
	void fetchMatrix(const std_msgs::Float64MultiArray::ConstPtr& mat);

private:
	ScalarVisualization *m_scalarVisualization;
	MatrixVisualization *m_matrixVisualization;
	QList<double> m_dataList;
};

#endif // MATRIXFETCHER_H
#ifndef MATRIXVISUALIZATION_H
#define MATRIXVISUALIZATION_H

#include "datavisualization.h"
#include "rossession.h"

#include <QImage>
#include <QLabel>
#include <QVBoxLayout>


class MatrixVisualization : public DataVisualization
{
	Q_OBJECT
public:
	explicit MatrixVisualization(QWidget *parent = nullptr,
	                             ROSSession *rosSession = nullptr,
	                             QGraphicsScene *scene = nullptr,
	                             DiagramBox *box = nullptr);

private:
	QLabel *m_thermalImageLabel;
	QImage m_thermalImage;
	QVBoxLayout *m_vLayout;

private slots:
	void switchToThermal();
	void switchToImage();
	void switchToLandscape();
	void updateThermal(QList<double> *values);
};

#endif // MATRIXVISUALIZATION_H
#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <QApplication>
#include <QString>

void coloredMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg);

#endif //MESSAGE_HANDLER_H
#ifndef MOVECOMMAN_H
#define MOVECOMMAN_H

#include "diagrambox.h"

#include <QUndoCommand>

/**
 * @brief The MoveCommand class represents moving a @DiagramBox on a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */

class MoveCommand : public QUndoCommand
{
public:
	MoveCommand(DiagramBox *box, const QPointF &oldPos, QUndoCommand *parent = nullptr);
	void undo() override;
	void redo() override;

private:
	DiagramBox *m_box;
	QPointF m_oldPos;
	QPointF m_newPos;
};

#endif // MOVECOMMAN_H
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
#ifndef OUTPUTSLOT_H
#define OUTPUTSLOT_H

#include "slot.h"
#include "types.h"

#include <QString>
#include <set>

/**
 * @brief The specialized version of @Slot used for output.
 * They contain @Link s that goes out of a @DiagramBox.
 * Outputs slots react to the mouse by growing sightly when the mouse comes nears them, in order
 * to facilitate the creation of a link.
 */

class Link;

class OutputSlot : public Slot
{
    Q_OBJECT
public:
    explicit OutputSlot();

    std::set<Link *> outputs() const;

    void addOutput(Link *output);
    void removeOutput(Link *output);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QRectF boundingRect() const override;

    void mousePressEvent(QGraphicsSceneMouseEvent *evt);
    void hoverEnterEvent(QGraphicsSceneHoverEvent *evt);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *evt);

    bool isDrawingLine() const;
    void setIsDrawingLine(bool isDrawingLine);

    void updateLinks();

    OutputType outputType() const;
    void setOutputType(const OutputType &outputType);

private:
    std::set<Link *> m_outputs; // The set of links which leaves this slot
    bool m_isDrawingLine;       // Indicate if we are drawing an outgoing link
    OutputType m_outputType;    // Indicate whether this function (slot) outputs a matrix or scalar
};

#endif // OUTPUTSLOT_H
#ifndef PAPYRUSWINDOW_H
#define PAPYRUSWINDOW_H

#include "librarypanel.h"
#include "library.h"
#include "script.h"
#include "propertiespanel.h"
#include "rosnode.h"
#include "homepage.h"
#include "rossession.h"
#include "types.h"
#include "xmldescriptionreader.h"
#include "token.h"

#include <QMainWindow>
#include <QGraphicsScene>
#include <QDir>
#include <QSystemTrayIcon>
#include <QLineEdit>
#include <QLabel>
#include <QAction>
#include <QTimer>

namespace Ui {
class PapyrusWindow;
}

// Define the type of development environment (and where to go look for libraries)
enum DevelopmentType {
	RELEASE,
	DEBUG
};
Q_DECLARE_METATYPE(DevelopmentType) // This allows convertion from/to QVariant

// Define if we are asking the user for the DESCRIPTION or the LIBRARY path
enum PathType {
	PATH_DESC,
	PATH_LIB
};
Q_DECLARE_METATYPE(PathType);

/**
 * @brief The PapyrusWindow class is the main window of the application.
 * It contains the list of open @Script s, the @Library of @Function s,
 * the @DiagramView s, etc.
 */
class PapyrusWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit PapyrusWindow(int argc, char **argv, QWidget *parent = 0);
	~PapyrusWindow();

	void closeEvent(QCloseEvent *evt);

	void readSettings(QString &lastOpenedScripts, int *lastActiveScript);
	void writeSettings();
	Script *parseXmlScriptFile(const QString &scriptPath);
	void askForPath(bool displayWarning, const PathType &pathType);
	void parseOneLevel(QDir dir, XmlDescriptionReader *xmlReader);
	QString getDescriptionPath();
	QString getLibPath();
	void updateButtonsState();

	QDir description() const {return description_;}
	void setDescription(QDir description) {description_ = description;}

	Category *addTreeRoot(QString name);

	QLineEdit *librarySearchField() const;
	void setLibrarySearchField(QLineEdit *librarySearchField);

	Ui::PapyrusWindow *ui() const;

	Library *getLibrary() const;
	void setLibrary(Library *library);

	std::set<Script *> getScripts() const;
	void addScript(Script *script);

	Script *activeScript() const;

	PropertiesPanel *propertiesPanel() const;
	void setPropertiesPanel(PropertiesPanel *propertiesPanel);

	QSystemTrayIcon *getTrayIcon() const;

	RosNode *rosnode() const;
	void setRosnode(RosNode *rosnode);

	void spawnRosNode();

	HomePage *homePage() const;
	void setHomePage(HomePage *homePage);

	DevelopmentType developmentType() const;

	QString debugPath() const;

	QString releasePath() const;

	QString debugLibPath() const;

	QString releaseLibPath() const;

	QString keyFile() const;
	void setKeyFile(const QString &keyFile);

	QString ivFile() const;
	void setIvFile(const QString &ivFile);

	QString lastDir() const;
	void setLastDir(const QString &lastDir);

	QTimer *autoSaveTimer() const;
	void setAutoSaveTimer(QTimer *autoSaveTimer);

private:
	Ui::PapyrusWindow *m_ui;
	RosNode *m_rosnode;
	int m_argc;
	char **m_argv;
	QLabel *m_rosMasterStatus;
	LibraryPanel *libraryPanel_;
	QLineEdit *librarySearchField_;
	QString m_lastExpandedCategory;  // Name of the last category that was expanded before filtering
	int m_libraryParsingErrors;
	QDir description_;
	QSystemTrayIcon *trayIcon;
	Library *m_library;
	std::set<Script *> m_scripts;
	Script *m_activeScript;
	PropertiesPanel *m_propertiesPanel;
	HomePage *m_homePage;
	QLineEdit *m_runTimeDisplay;
	DevelopmentType m_developmentType;
	QAction *m_actionRelease;
	QAction *m_actionDebug;
	QString m_debugPath;        // Path where to search for description files in DEBUG mode
	QString m_releasePath;      // Path where to search for description files in RELEASE mode
	QString m_debugLibPath;     // Path where to search for library files in DEBUG mode
	QString m_releaseLibPath;   // Path where to search for library files in RELEASE mode
	QString m_keyFile;          // Path of the key file to crypt / decrypt scrip files
	QString m_ivFile;           // Path of the IV
	QString m_lastDir;          // Last directory visited for saving or loading files
	QTimer *m_autoSaveTimer;    // Timer to trigger auto save for scripts
	QString m_changelogVersion; // Used to know if we should show the changelog on launch
	QTimer *m_checkVersionTimer; // Timer that periodically check for new version release
	bool m_preventROSPopup;     // Prevents displaying ROS master pop-ups

signals:
	void toggleDisplayGrid(bool);
	void launched();

private slots:
	void filterLibraryNames(const QString &text);
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void onROSMasterChange(bool isOnline);
	void onScriptResumed();
	void onScriptPaused();
	void onScriptStopped();
	void updateStopWatch(int h, int m, int s, int ms);
	void updateDevelopmentEnvironment(QAction *action);
	void categoryExpanded(QTreeWidgetItem *item);
	void autoSave();
	void onPropPanelEnter();
	void onPropPanelEscape();
	void onLaunched();
	void openScript(QString path = "");
	void checkForNewRelease();
	void reEnableROSPopUp();

	void on_actionExit_triggered();

	void on_actionAntialiasing_toggled(bool antialiasing);

	void on_actionZoom_In_triggered();

	void on_actionZoom_Out_triggered();

	void on_actionZoom_Fit_triggered();

	void on_actionNew_script_hovered();

	void on_actionOpen_Script_hovered();

	void on_actionSave_Script_hovered();

	void on_actionZoom_In_hovered();

	void on_actionZoom_Out_hovered();

	void on_actionZoom_Fit_hovered();

	void on_actionNew_script_triggered();

	void on_actionDisplay_Grid_hovered();

	void on_actionDisplay_Grid_toggled(bool arg1);

	void on_actionAbout_Papyrus_triggered();

	void on_actionSave_Script_triggered();

	void on_actionOpen_Script_triggered();
	void on_tabWidget_currentChanged(int index);
	void on_tabWidget_tabBarDoubleClicked(int index);
	void on_actionClose_Script_triggered();
	void on_actionConnect_triggered();
	void on_actionRun_triggered();
	void on_actionStop_triggered();
	void on_actionScope_triggered();
	void on_actionEdit_paths_triggered();
	void on_actionShow_all_outputs_triggered();
	void on_actionHide_all_outputs_triggered();
	void on_actionList_shortcuts_triggered();
	void on_actionChangelog_triggered(bool isNewRelease = false);
	void on_actionReopen_last_scripts_triggered();
	void on_actionUndo_triggered();
	void on_actionRedo_triggered();
};

#endif // PAPYRUSWINDOW_H
#ifndef PROPDOUBLESPINBOX_H
#define PROPDOUBLESPINBOX_H

#include <QDoubleSpinBox>

/**
 * @brief The PropDoubleSpinBox class is the standard QDoubleSpinBox with only one minor
 * modification: its sizeHint has been made editable
 */

class PropDoubleSpinBox : public QDoubleSpinBox
{
public:
	PropDoubleSpinBox(QWidget *parent = nullptr);

	QSize sizeHint() const override;
	void setSizeHint(const QSize &sizeHint);

private:
	QSize m_sizeHint;
};

#endif // PROPDOUBLESPINBOX_H
#ifndef PROPERTIESPANEL_H
#define PROPERTIESPANEL_H

#include "diagrambox.h"
#include "link.h"
#include "script.h"
#include "zone.h"
#include "setcolorbutton.h"
#include "proplineedit.h"
#include "propdoublespinbox.h"

#include <QGroupBox>
#include <QLineEdit>
#include <QFrame>
#include <QLabel>
#include <QSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QFormLayout>

/**
 * @brief The PropertiesPanel class is used to display (and modify) the properties of selected
 * objects (such as slots, boxes, links, etc.)
 */

class PropertiesPanel : public QGroupBox
{
	Q_OBJECT

public:
	explicit PropertiesPanel(QWidget *parent = 0);

	void hideAllFrames(bool buttonsToo = false);
	void updateBoxProperties(DiagramBox *box);
	void updateLinkProperties(Link *link);
	void updateScriptProperties(Script *script);
	void updateZoneProperties(Zone *zone);

	void keyPressEvent(QKeyEvent *event);

	QFrame *boxFrame() const;
	void setBoxFrame(QFrame *boxFrame);

	QFrame *linkFrame() const;
	void setLinkFrame(QFrame *linkFrame);

	QLabel *boxName() const;
	void setBoxName(QLabel *boxName);

	QSpinBox *rowsInput() const;

	QSpinBox *colsInput() const;

	QPushButton *okBtn() const;
	void setOkBtn(QPushButton *okBtn);

	QPushButton *cancelBtn() const;
	void setCancelBtn(QPushButton *cancelBtn);

	QCheckBox *saveActivity() const;
	void setSaveActivity(QCheckBox *saveActivity);

	QDoubleSpinBox *linkWeight() const;

	QLabel *scriptName() const;
	void setScriptName(QLabel *scriptName);

	PropDoubleSpinBox *timeValue() const;
	void setTimeValue(PropDoubleSpinBox *timeValue);

	QComboBox *timeUnit() const;
	void setTimeUnit(QComboBox *timeUnit);

	QLineEdit *linkValue() const;
	void setLinkValue(QLineEdit *linkValue);

	QPushButton *displayVisu() const;
	void setDisplayVisu(QPushButton *displayVisu);

	PropLineEdit *zoneTitle() const;
	void setZoneTitle(PropLineEdit *zoneTitle);

	SetColorButton *zoneColor() const;
	void setZoneColor(SetColorButton *zoneColor);

	PropLineEdit *boxTitle() const;
	void setBoxTitle(PropLineEdit *boxTitle);

	QLabel *boxMatrixShape() const;
	void setBoxMatrixShape(QLabel *boxMatrixShape);

	QCheckBox *publish() const;
	void setPublish(QCheckBox *publish);

	PropLineEdit *topic() const;
	void setTopic(PropLineEdit *topic);

	QCheckBox *linkSecondary() const;
	void setLinkSecondary(QCheckBox *linkSecondary);

	QCheckBox *encrypt() const;
	void setEncrypt(QCheckBox *encrypt);

private:
	QVBoxLayout *m_panelLayout;  // The properties panel's main layout
	QFrame *m_scriptFrame;       // Container for script's properties
	QLabel *m_scriptName;        // Label used to change the script (and tab) name
	QLabel *m_timeLabel;         // Contains either "frequency" or "period"
	PropDoubleSpinBox *m_timeValue; // Used to input the script's frequency (or period)
	QComboBox *m_timeUnit;       // Used to select the unit (in Hz or ms)
	QCheckBox *m_encrypt;        // Whether or not the file is encrypted on save

	QFormLayout *m_boxLayout;  // Layout for the box properties (access needed to hide rows)
	QFrame *m_boxFrame;        // Container for box's properties
	QLabel *m_boxName;         // Display the name of the box
	PropLineEdit *m_boxTitle;     // Allow to see or change the box's custom name
	QLabel *m_boxOutputType;   // Display the box's output type (scalar, matrix)
	QLabel *m_boxMatrixShape;  // Display the shape of the function (when matrix)
	QSpinBox *m_rowsInput;     // Spin box to input number of rows in the output (if matrix)
	QSpinBox *m_colsInput;     // Spin box to input number of columns in the output (if matrix)
	QCheckBox *m_saveActivity; // To enable saving the activity of the box
	QCheckBox *m_publish;      // To enable publish the output of the function
	PropLineEdit *m_topic;        // To input the topic name for publishing
	QPushButton *m_displayVisu; // (TEMP) display the box's data vizualisation

	QFormLayout *m_linkLayout;    // Layout for the link properties (access needed to hide rows)
	QFrame *m_linkFrame;          // Container for link's properties
	QLabel *m_linkType;           // Display the type of the link
	QCheckBox *m_linkSecondary;   // Will display if the link is secondary or not
	QDoubleSpinBox *m_linkWeight; // Spin box to set the weight of the link
	QLineEdit *m_linkValue;       // Text field to enter the link's value (for string links)

	QFormLayout *m_zoneLayout;     // Contains the layout to display comment zone's properties
	QFrame *m_zoneFrame;           // Container for zone's properties
	PropLineEdit *m_zoneTitle;        // The comment zone's title
	SetColorButton *m_zoneColor;     // Holds the color of the comment zone

	// Will contain the sizes of connected matrix when the link is SPARSE_MATRIX
	QSize m_inputSize;
	QSize m_outputSize;

	QPushButton *m_okBtn;      // Button used to validate changes in parameters
	QPushButton *m_cancelBtn;  // Button used to discard changes in parameters and restore current

public slots:
	void displayBoxProperties(DiagramBox *box);
	void displayLinkProperties(Link *link);
	void displayScriptProperties(Script *script);
	void displayZoneProperties(Zone *zone);
	void convertTimeValues(int idx);
	void toggleTopic(bool isChecked);

private slots:
	void onTopicChanged(const QString &topic);

signals:
	void enterPressed();
	void escapePressed();
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
};

#endif // PROPERTIESPANEL_H
#ifndef PROPLINEEDIT_H
#define PROPLINEEDIT_H

#include <QLineEdit>

/**
 * @brief The PropLineEdit class is a standard @QLineEdit with only one minor modification: its
 * sizeHint can be changed
 */

class PropLineEdit : public QLineEdit
{
public:
	PropLineEdit(QWidget *parent = nullptr);
	PropLineEdit(const QString &contents, QWidget *parent = nullptr);

	QSize sizeHint() const override;
	void setSizeHint(const QSize &sizeHint);

private:
	QSize m_sizeHint; // Modified sizeHint
};

#endif // PROPLINEEDIT_H
#ifndef ROSNODE_H
#define ROSNODE_H

#include <QThread>

#include <ros/ros.h>

/**
 * @brief The RosNode class is used to handle ROS
 * operations
 */
class RosNode : public QThread
{
	Q_OBJECT
public:
	RosNode(int argc, char **argv);
	virtual ~RosNode();

	void init();
	void run() override;

	bool shouldQuit() const;
	void setShouldQuit(bool value);

private:
	int m_argc;
	char **m_argv;
	ros::Subscriber m_sub;
	bool m_shouldQuit;       // Used by Papyrus to cleanly exit the thread

signals:
	void rosMasterChanged(bool isOnline);
};

#endif // ROSNODE_H
#ifndef ROSSESSION_H
#define ROSSESSION_H

#include <ros/ros.h>

//#include "script.h"
#include "types.h"

#include <QThread>
#include <QString>
#include <QSet>

#include "diagnostic_msgs/KeyValue.h"

/**
 * @brief The ROSSession class contains parameters related to the current ROS session (connection
 * with a Kheops node
 */
class ROSSession : public QThread
{
	Q_OBJECT

public:
	ROSSession(const QString &nodeName, QObject *parent = nullptr);
	~ROSSession();

	void addToHotList(QUuid uuid);

	bool shouldQuit() const;
	void setShouldQuit(bool shouldQuit);

	QString nodeName() const;
	void setNodeName(const QString &nodeName);

private:
	bool m_shouldQuit;       // Used to properly exit the thread
	QString m_nodeName;      // Name of the node it should listen
	QSet<QUuid> m_hotList;   // List of functions' uuids whose output to activate on-the-fly
	bool m_isFirstRun;       // To differentiate a 'resume' on first launch or after a pause

	void run() override;
	void handleStatusChange(const diagnostic_msgs::KeyValue::ConstPtr& msg);
	void activateOutput(QUuid uuid);

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void scriptResumed();
	void scriptPaused();
	void scriptStopped();
};

#endif // ROSSESSION_H
#ifndef SCALARFETCHER_H
#define SCALARFETCHER_H

#include "datafetcher.h"
#include "types.h"

#include <QDebug>
#include <QBarSet>
#include <QSplineSeries>

#include <ros/ros.h>
#include "std_msgs/Float64.h"

QT_CHARTS_USE_NAMESPACE

class ScalarVisualization;

class ScalarFetcher : public DataFetcher
{
	Q_OBJECT
public:
	explicit ScalarFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent = nullptr);
	explicit ScalarFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, VisualizationType type, QObject *parent = nullptr);

	void setVisType(VisualizationType type) override;

protected:
	void run() override;

	void fetchScalar(const std_msgs::Float64::ConstPtr& scalar);

private:
	ScalarVisualization *m_scalarVisualization;
};

#endif // SCALARFETCHER_H
#ifndef SCALARVISUALIZATION_H
#define SCALARVISUALIZATION_H

#include "types.h"
#include "datavisualization.h"
#include "rossession.h"

#include <QBarSet>
#include <QBarSeries>
#include <QHorizontalBarSeries>
#include <QChart>
#include <QChartView>
#include <QSplineSeries>
#include <QValueAxis>
#include <QVBoxLayout>
#include <QList>

#include <vector>

QT_CHARTS_USE_NAMESPACE

class ScalarVisualization : public DataVisualization
{
	Q_OBJECT
public:
	ScalarVisualization(QWidget *parent = nullptr,
	                    ROSSession *rosSession = nullptr,
	                    QGraphicsScene *scene = nullptr,
	                    DiagramBox *box = nullptr);
	void mousePressEvent(QMouseEvent *evt);

	void updateBarValues(const std::vector<qreal> &values);
	void pushGraphValues(const std::vector<qreal> &values);

protected:
	void createCharts();

	int m_size;
	int m_idx;

	QBarSet *m_barSet;
	QBarSeries *m_barSeries;
	QHorizontalBarSeries *m_horizontalBarSeries;
	QChart *m_barChart;
	QValueAxis *m_barAxisY;
	QValueAxis *m_barAxisX;
	QChartView *m_barView;
	qreal m_barMin;
	qreal m_barMax;
	MatrixShape m_matrixShape;  // used to determine wether to display a ROW or COL vector (when applicable)

	QList<QSplineSeries *> m_graphSeries;
	QChart *m_graphChart;
	QVBoxLayout *m_vLayout;
	QValueAxis *m_graphAxisX;
	QValueAxis *m_graphAxisY;
	QChartView *m_graphView;
	qreal m_graphMin;
	qreal m_graphMax;

protected slots:
	void switchToBar();
	void switchToGraph();
};

#endif // SCALARVISUALIZATION_H
#ifndef SCRIPT_H
#define SCRIPT_H

#include "diagramscene.h"
#include "types.h"
#include "rossession.h"

#include <QString>
#include <QFile>
#include <QDir>
#include <QUuid>
#include <QTimer>

enum ScriptStatus {
	INVALID_SCRIPT_STATUS,
	SCRIPT_RUNNING,
	SCRIPT_PAUSED
};

// Forward declaration because of recursive includes
class DiagramScene;

Q_DECLARE_METATYPE(TimeUnit) // This allows convertion from/to QVariant

/**
 * @brief The Script class represents a neural scripts. An XML file is associated to it,
 * as well as the @QGraphicsScene that contains its functions boxes
 */

class Script : public QObject
{
	Q_OBJECT
public:
	Script(DiagramScene *scene, const QString &name = "");
	~Script();

	void save(const QString &basePath = QDir::homePath(),
	          bool isAutoSave = false);

	void updateTextStyle();
	void runOrPause();
	void run();
	void pause();
	void stop();
	ScriptStatus queryScriptStatus();
	void setupROSSession();

	QString name() const;
	void setName(const QString &name);

	QString filePath() const;
	void setFilePath(const QString &filePath);

	DiagramScene *scene() const;

	bool modified() const;

	void setStatusModified(bool isModified);

	bool isInvalid() const;
	void setIsInvalid(bool isInvalid);

	double timeValue() const;
	void setTimeValue(double timeValue);

	TimeUnit timeUnit() const;
	void setTimeUnit(const TimeUnit &timeUnit);

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	bool encrypt() const;
	void setEncrypt(bool encrypt);

	ROSSession *rosSession() const;
	void setRosSession(ROSSession *rosSession);

	bool isActiveScript() const;
	void setIsActiveScript(bool isActiveScript);

	bool hasTab() const;
	void setHasTab(bool hasTab);

	bool isRunning() const;
	void setIsRunning(bool isRunning);

	bool isPaused() const;
	void setIsPaused(bool isPaused);

	QString nodeName() const;
	void setNodeName(const QString &nodeName);

public slots:
	void warnAboutModifiedScript();

private:
	DiagramScene *m_scene; // The associated scene for this script
	bool m_hasTab; // Tells whether the scripts has a tab in the tabwidget or not
	ROSSession *m_rosSession; // The associated ROS Session for this script
	QString m_name;        // Pretty name of the script (to display in tabs for instance)
	QString m_nodeName;    // The ROS node name of the script
	QString m_filePath;    // Path of the (XML) file in which to save this script
	bool m_modified;       // Whether there was some changes since last save
	bool m_isInvalid;      // Whether this script is currently invalid (and thus prevent saving)
	double m_timeValue;    // The RT Token time (either frequency or period)
	TimeUnit m_timeUnit;   // Whether the time value is a frequency or a period
	QUuid m_uuid;          // UUID for the RT Token (needed by kheops)
	QTimer *m_modifiedNotifTimer; // Timer to display a system tray notification when unsaved for more than X minutes
	bool m_encrypt;        // Whether the XML script should be encrypted on save (to protect IP)
	std::string m_key;     // AES Key used to encrypt the file
	std::string m_iv;      // AES IV used to encrypt the file
	bool m_isActiveScript; // Tells this script if it's the currently active one
	bool m_isRunning;      // Tells whether this script is running (launched)
	bool m_isPaused;       // Tells whether this script is paused while running

private slots:
	void onROSSessionMessage(const QString &msg, MessageUrgency urgency = MSG_INFO);
	void onScriptResumed();
	void onScriptPaused();
	void onScriptStopped();
	void onTimeElapsed(int h, int m, int s, int ms);
//	void temporaryCheckLaunch();

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void scriptResumed();
	void scriptPaused();
	void scriptStopped();
	void timeElapsed(int h, int m, int s, int ms);
};

#endif // SCRIPT_H
#ifndef SETCOLORBUTTON_H
#define SETCOLORBUTTON_H

#include <QPushButton>
#include <QColor>

class SetColorButton : public QPushButton
{
	Q_OBJECT

public:
	explicit SetColorButton(QWidget *parent = nullptr);

	void updateColor();

	QColor color() const;
	void setColor(const QColor &color);

private:
	QColor m_color;

private slots:
	void changeColor();
};

#endif // SETCOLORBUTTON_H
#ifndef SLOT_H
#define SLOT_H

#include <QObject>
#include <QString>
#include <QGraphicsItem>

//#include <diagrambox.h>
class DiagramBox;

/**
 * @brief The Slot class defines an argument slot, i.e. an item will either receive
 * a connection from another box's output slot, or from which a connection leaves to
 * reach another box' input slot.
 * This class is meant to be subclassed (see @InputSlot and @OutputSlot).
 */

/*
enum InputType {
    SCALAR_SCALAR,
    SCALAR_MATRIX,
    MATRIX_MATRIX,
    SPARSE_MATRIX
};

enum OutputType {
    SCALAR,
    MATRIX
};
*/

class Slot : public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    explicit Slot(QGraphicsItem *parent = 0);
    explicit Slot(QString &name, QGraphicsItem *parent = 0);
    ~Slot();

    QString name() const;
    void setName(const QString &name);

    virtual QRectF boundingRect() const = 0;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) = 0;

    qreal dist() const;
    void setDist(const qreal &dist);

    DiagramBox *box() const;
    void setBox(DiagramBox *box);

protected:
    QString m_name;    // The name of this slot
    qreal m_dist;      // Distance to the mouse (used to highlight the slot when mouse approach)
    DiagramBox *m_box; // The DiagramBox that is associated with this Slot
};

#endif // SLOT_H
#ifndef SWAPBOXESCOMMAND_H
#define SWAPBOXESCOMMAND_H

#include "diagrambox.h"
#include "diagramscene.h"
#include "link.h"

#include <QUndoCommand>

/**
 * @brief The SwapBoxesCommand class represents swapping two functions. This is used to provide
 * Undo/Redo functionality.
 */

class SwapBoxesCommand : public QUndoCommand
{
public:
	SwapBoxesCommand(DiagramScene *scene, DiagramBox *toSwap, DiagramBox *newBox,
	                 QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	DiagramScene *m_scene;  // The scene in which the swap takes place
	DiagramBox *m_toSwap;   // The box to be swapped
	DiagramBox *m_newBox;   // The box to add

	QList<Link *> m_outputLinks; // List of all links that are FROM m_toSwap
	QHash<QString, QList<Link *>> m_inputLinks; // List of all links that are TO m_toSwap, sorted by input slots
};

#endif // SWAPBOXESCOMMAND_H
#ifndef TOKEN_H
#define TOKEN_H

// Define token and password to check Papyrus version on gitlab
#define GITLAB_TOKEN "gitlab+deploy-token-1"
#define GITLAB_PWD "xz7LJDr-bMe6wZYHzLze"

#endif // TOKEN_H
#ifndef TYPES_H
#define TYPES_H

/**
 * This is just a collection of types (no functions!) in order to simply the include process
 */

// Define the type that an @InputSlot accepts
enum InputType{
	INVALID_INPUT_TYPE,
	SCALAR_SCALAR,
	SCALAR_MATRIX,
	MATRIX_MATRIX,
	STRING_INPUT
};

// Define the type that an @OutputSlot (hence a function, really) outputs
enum OutputType{
	INVALID_OUTPUT_TYPE,
	SCALAR,
	MATRIX,
	STRING
};

// Define the shape of a matrix-typed @DiagramBox
enum MatrixShape {
	INVALID_MATRIX_SHAPE,
	SHAPE_NONE,
	POINT,
	VECT,
	ROW_VECT,
	COL_VECT
};

// Define whether the time value is a frequency in Hz or a period in ms for a @Script
enum TimeUnit{
	INVALID_UNIT,
	HZ,
	MS
};

// Define the urgency of a message that will be shown on the status bar (and then its color)
enum MessageUrgency {
	INVALID_MESSAGE_URGENCY,
	MSG_INFO,
	MSG_WARNING,
	MSG_ERROR
};

// Define the type of connectivity for MATRIX_MATRIX links
enum Connectivity {
	INVALID_CONNECTIVITY,
	ONE_TO_ONE,
	ONE_TO_ALL,
	ONE_TO_NEI
};

// Define the type of visualization for data
enum VisualizationType {
	INVALID_VISUALIZATION_TYPE,
	BAR,        // scalar and vector
	GRAPH,      // scalar and vector
	IMAGE,      // matrix
	GRAYSCALE,  // matrix
	LANDSCAPE   // matrix
};

// Define the different reasons why a Link can be invalid
enum LinkInvalidReason {
	INVALID_INVALID_REASON = 0, // MUST be zero (because it uses bitwise OR)
	TYPES_INCOMPATIBLE     = 0b1,
	SIZES_DONT_MATCH       = 0b10,
	SHAPE_MUST_BE_POINT    = 0b100,
	SHAPE_MUST_BE_VECT     = 0b1000,
	SHAPE_MUST_BE_ROW_VECT = 0b10000,
	SHAPE_MUST_BE_COL_VECT = 0b100000
};

// Defining bitwise operations for LinkInvalidReason because in C++11, enums are scoped
inline LinkInvalidReason operator|(LinkInvalidReason a, LinkInvalidReason b) {
	return static_cast<LinkInvalidReason>(static_cast<int>(a) | static_cast<int>(b));
}

inline LinkInvalidReason operator&(LinkInvalidReason a, LinkInvalidReason b) {
	return static_cast<LinkInvalidReason>(static_cast<int>(a) & static_cast<int>(b));
}

// Define the different reasons why a box can be invalid
enum BoxInvalidReason {
	INVALID_BOX_INVALID_REASON = 0, // MUST be zero (because it uses bitwise OR)
	INPUT_FULL                 = 0b1,
	BOX_MUST_BE_POINT          = 0b10,
	BOX_MUST_BE_VECT           = 0b100,
	BOX_MUST_BE_ROW_VECT       = 0b1000,
	BOX_MUST_BE_COL_VECT       = 0b10000,
};

// Defining bitwise operations for BoxInvalidReason because in C++11, enums are scoped
inline BoxInvalidReason operator|(BoxInvalidReason a, BoxInvalidReason b) {
	return static_cast<BoxInvalidReason>(static_cast<int>(a) | static_cast<int>(b));
}

inline BoxInvalidReason operator&(BoxInvalidReason a, BoxInvalidReason b) {
	return static_cast<BoxInvalidReason>(static_cast<int>(a) & static_cast<int>(b));
}

#endif // TYPES_H
#ifndef UPDATEBOXCOMMAND_H
#define UPDATEBOXCOMMAND_H

#include "propertiespanel.h"
#include "diagrambox.h"

#include <QUndoCommand>

/**
 * @brief The UpdateZoneCommand class represents updating a @DiagramBox's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateBoxCommand : public QUndoCommand
{
public:
	UpdateBoxCommand(PropertiesPanel *panel, DiagramBox *box, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	DiagramBox *m_box;        // The box which we are modifying parameters

	// Old parameters (ones the box had before updating its parameters)
	QString m_oldTitle;
	int m_oldRows;
	int m_oldCols;
	bool m_oldActivity;
	bool m_oldPublish;
	QString m_oldTopic;

	// New parameters (ones the box will have after updating its parameters)
	QString m_newTitle;
	int m_newRows;
	int m_newCols;
	bool m_newActivity;
	bool m_newPublish;
	QString m_newTopic;
};

#endif // UPDATEBOXCOMMAND_H
#ifndef UPDATELINKCOMMAND_H
#define UPDATELINKCOMMAND_H

#include "propertiespanel.h"
#include "link.h"

#include <QUndoCommand>

/**
 * @brief The UpdateLinkCommand class represents updating a @Link's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateLinkCommand : public QUndoCommand
{
public:
	UpdateLinkCommand(PropertiesPanel *panel, Link *link, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	Link *m_link;             // The link which we are modifying parameters

	// Old parameters
	qreal m_oldWeight;
	QString m_oldValue;
	bool m_oldSecondary;

	// New parameters
	qreal m_newWeight;
	QString m_newValue;
	bool m_newSecondary;
};

#endif // UPDATELINKCOMMAND_H
#ifndef UPDATESCRIPTCOMMAND_H
#define UPDATESCRIPTCOMMAND_H

#include "propertiespanel.h"
#include "script.h"

#include <QUndoCommand>

/**
 * @brief The UpdateScriptCommand class represents updating a @Script's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateScriptCommand : public QUndoCommand
{
public:
        UpdateScriptCommand(PropertiesPanel *panel, Script *script, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	Script *m_script;           // The script which we are modifying parameters

	// Old parameters
	qreal m_oldTime;
	TimeUnit m_oldUnit;
	bool m_oldEncrypt;

	// New parameters
	qreal m_newTime;
	TimeUnit m_newUnit;
	bool m_newEncrypt;
};

#endif // UPDATESCRIPTCOMMAND_H
#ifndef UPDATEZONECOMMAND_H
#define UPDATEZONECOMMAND_H

#include "propertiespanel.h"
#include "zone.h"

#include <QUndoCommand>

/**
 * @brief The UpdateZoneCommand class represents updating a @Zone's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateZoneCommand : public QUndoCommand
{
public:
	UpdateZoneCommand(PropertiesPanel *panel, Zone *zone, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	Zone *m_zone;             // The zone which we are modifying parameters

	// Old parameters
	QString m_oldTitle;
	QColor m_oldColor;

	// New parameters
	QString m_newTitle;
	QColor m_newColor;
};

#endif // UPDATEZONECOMMAND_H
#ifndef XMLDESCRIPTIONREADER_H
#define XMLDESCRIPTIONREADER_H

#include "category.h"
#include "slot.h"

#include <QXmlStreamReader>

/**
 * @brief The XmlDescriptionReader class is a XML file parser whose task is to populate a @Library
 * object with the @Categorie  and descriptions of neural @Function s, read from their XML description
 * file.
 */

class XmlDescriptionReader
{
public:
	XmlDescriptionReader(Category *category );

	bool read(QIODevice *device, const QString &descriptionFile);
private:
//    Library *m_library;
	Category *m_category;
	QXmlStreamReader reader;

	void readDescription(const QString &descriptionFile);
	void readAllFunctions(const QString &libName, const QString &descriptionFile);
	void readOneFunction(const QString &libName, const QString &descriptionFile);
	void readName(Function *function);
	void readInputs(Function *function);
	QString readIcon();
	void readParameterName(Slot *paramSlot);
	void readParameterType(OutputSlot *paramSlot);
	void readParameterType(InputSlot *paramSlot);
	void readParameterDesc(InputSlot *paramSlot);
	void readOutput(Function *function);
	void readFunctionDesc(Function *function);
};

#endif // XMLDESCRIPTIONREADER_H
#ifndef XMLSCRIPTREADER_H
#define XMLSCRIPTREADER_H

#include "script.h"
#include "diagrambox.h"
#include "outputslot.h"
#include "inputslot.h"
#include "library.h"

#include <QXmlStreamReader>
#include <QUuid>

/**
 * @brief The XmlScriptReader class is an XML parser that reads an XML @Script file
 * to populate the @GraphicsScene.
 */

class XmlScriptReader
{
public:
	explicit XmlScriptReader(Script *script, const QString &descriptionPath, Library *library);
	bool read(QIODevice *device);

	QString errorString() const;

	QPointF centerView() const;
	void setCenterView(const QPointF &centerView);

	Library *library() const;
	void setLibrary(Library *library);

private:
	QXmlStreamReader reader;
	QString m_errorString;
	Script *m_script;
	QString m_descriptionPath;
	QPointF m_centerView;
	Library *m_library;

	void readScript();
	void readFunction(std::map<QUuid, DiagramBox *> *allBoxes,
	                  std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readFunctionName(QString &name);
	void readFunctionTitle(QString &title);
	void readFunctionSave(bool *save);
	void readPublishTopic(QString &topic, bool *publish);
	void readInputSlots(std::vector<InputSlot *> *inputSlots,
	                    std::map<QUuid, DiagramBox *> *allBoxes,
	                    std::set<std::pair<QUuid, Link *>> *incompleteLinks);
	void readOutputSlot(OutputSlot *outputSlot, int *rows, int *cols);
	void readUUID(QUuid *uuid);
	void readPosition(QPointF *pos);
	void readLinks(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
	               std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readLink(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
	              std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readZone();
};

#endif // XMLSCRIPTREADER_H
#ifndef ZONE_H
#define ZONE_H

#include <QGraphicsObject>
#include <QGraphicsItem>
#include <QColor>
#include <QGraphicsItemGroup>

enum ResizeType {
	NO_RESIZE,
	RESIZE_TOP,
	RESIZE_RIGHT,
	RESIZE_BOTTOM,
	RESIZE_LEFT
};

class Zone : public QGraphicsObject
{
public:
	explicit Zone(QGraphicsObject *parent = nullptr);
	explicit Zone(qreal x, qreal y, qreal w, qreal h, QGraphicsObject *parent = nullptr);
	~Zone();

	QRectF boundingRect() const override;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	QVariant itemChange(GraphicsItemChange change, const QVariant &value);
	void mousePressEvent(QGraphicsSceneMouseEvent *event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	void hoverMoveEvent(QGraphicsSceneHoverEvent *event);

	void updateGroup(bool startFromScratch = false);
	void updateLinks();

	// Getters / Setters

	qreal width() const;
	void setWidth(const qreal &width);

	qreal height() const;
	void setHeight(const qreal &height);

	QColor color() const;
	void setColor(const QColor &color);

	QString title() const;
	void setTitle(const QString &title);

private:
	qreal m_width;
	qreal m_height;
	QColor m_color;
	QString m_title; // The title of the comment zone (should be kept small)
	ResizeType m_resizeType;
};

#endif // ZONE_H
