#include "helpers.h"

#include <unistd.h>
#include <sys/stat.h>

#include <QMessageBox>

/**
 * @brief getMainWindow returns the instance of the main Papyrus window, checking that is exists
 * @return
 */
PapyrusWindow *getMainWindow()
{
	PapyrusWindow *mainWindow = NULL;

	foreach (QWidget *w, qApp->topLevelWidgets()) {
		if (PapyrusWindow *mW = qobject_cast<PapyrusWindow *>(w)) {
			mainWindow = mW;
			break;
		}
	}

	if (mainWindow == NULL)
		informUserAndCrash(QObject::tr("Could not fetch Papyrus' main window."));

	return mainWindow;
}

/**
 * @brief outputTypeToString converts a value from the enum OutputType to a QString, used to save
 * script for instance
 * @param outputType the value to convert
 * @return
 */
QString outputTypeToString(const OutputType &outputType)
{
	switch (outputType) {
		case SCALAR:
		return QString("SCALAR");
		break;
		case MATRIX:
		return QString("MATRIX");
		break;
		case STRING:
		return QString("STRING");
		break;
		default:
			informUserAndCrash("Unsupported OutputType when converting to QString.");
		    return ""; // never reached, to prevent compiler warning
		break;
	}
}

/**
 * @brief stringToOutputType converts a QString to the enum OutputType. It is case-insensitive
 * @param str the string to convert
 * @return
 */
OutputType stringToOutputType(const QString &str)
{
	QString lower = str.toLower();

	if (lower == "scalar")
		return SCALAR;

	if (lower == "matrix")
		return MATRIX;

	// WARNING: both InputType and OutputType are written as "STRING" the XML files
	if (lower == "string")
		return STRING;

	informUserAndCrash("Failed to parse string into OutputType");
	return INVALID_OUTPUT_TYPE;
}

/**
 * @brief inputTypeToString converts a value from the enum InputType to a QString, used to save
 * script for instance
 * @param inputType the value to convert
 * @return
 */
QString inputTypeToString(const InputType &inputType)
{
	switch (inputType) {
		case SCALAR_SCALAR:
		return QString("SCALAR_SCALAR");
		break;
		case SCALAR_MATRIX:
		return QString("SCALAR_MATRIX");
		break;
		case MATRIX_MATRIX:
		return QString("MATRIX_MATRIX");
		break;
		case STRING_INPUT:
		return QString("STRING");
		break;
		default:
			informUserAndCrash("Unsupported InputType when converting to QString.");
		return ""; // never reached, to prevent compiler warning
		break;
	}
}

/**
 * @brief stringToInputType converts a string to the enum InputType. It is case-insensitive.
 * @param str the string to convert
 * @return
 */
InputType stringToInputType(const QString &str)
{
	QString lower = str.toLower();

	if (lower == "scalar_scalar")
		return SCALAR_SCALAR;

	if (lower == "scalar_matrix")
		return SCALAR_MATRIX;

	if (lower == "matrix_matrix")
		return MATRIX_MATRIX;

	// WARNING: both Input and Output types are written as "STRING" in XML files
	if (lower == "string")
		return STRING_INPUT;

	informUserAndCrash("Failed to parse string to InputType");

	return INVALID_INPUT_TYPE;
}

/**
 * @brief canLink returns whether the output type 'from' is compatible with input type 'to' in order
 * to create a link
 * @param from the @OutputType that we want to connect
 * @param to the @InputType we want to connect to
 * @return whether the connection is valid or not
 */
bool canLink(const OutputType &from, const InputType &to)
{
	// If the input slot expects a scalar, only a scalar can be connected to it
	if (from == SCALAR && to == SCALAR_SCALAR)
		return true;

	// If the input slot expects a matrix with any kind of connectivity, then only a matrix can
	// be connected
	if (from == MATRIX && to != SCALAR_SCALAR && to != STRING_INPUT)
		return true;

	// Match strings together
	if (from == STRING && to == STRING_INPUT)
		return true;

	// other cases are invalid
	return false;
}

/**
 * @brief informUserAndCrash is used whenever we perform a check (for instance check if a pointer is
 * null before using it (that's good practise BTW!)) and we then want to crash the application.
 * Rather that only writing a debug message in the logs (which the user probably would not read, and
 * this he will only pester agains the application, wondering what happened), the developper MUST
 * use this function with an informative error message.
 * This error message will be displayed on a blocking, modal window, with a unique button "close",
 * then it calls qFatal() which makes the application crash.
 * This way, the user knows why it crashed, and it's also logged.
 *
 * TL;DR: use this function *everywhere* you want to make the program crash
 * @param text the informative error message that will be displayed to the user and the logs
 * @param title (optional) the title of the modal window, will default to "Papyrus is about to crash!"
 */
void informUserAndCrash(const QString &text, const QString &title)
{
	QMessageBox::critical(NULL, title, text, QMessageBox::Close);
	qFatal("[Papyrus] FATAL: %s", text.toStdString().c_str());
}

/**
 * @brief rescaleSvgItem computes a ratio to apply to an svg icon, based on the given dimension.
 * If xOffset and/or yOffset are not null, the computed values for the offsets are set, so that
 * the user can then use these offsets to position the item at the center
 * @param svg the svg item to rescale
 * @param xOffset pointer to qreal to save xoffset
 * @param yOffset pointer to qreal to save yoffset
 */
void rescaleSvgItem(QGraphicsSvgItem *svg, const QSizeF &size, const QPointF &pos, bool center)
{
	QSizeF svgSize = svg->boundingRect().size();
	qreal targetWidth = size.width();
	qreal targetHeight = size.height();
	qreal svgWidth = svgSize.width();
	qreal svgHeight = svgSize.height();
	qreal xOffset = 0.0;
	qreal yOffset = 0.0;
	qreal ratio = 1.0;

	if (svgWidth > svgHeight) {
		// When scaling in width, we need to center the image vertically
		ratio = targetWidth / svgWidth;
		if (center)
			yOffset = (targetHeight - ratio * svgHeight) / 2.0;
	} else {
		// When scaling in height, we need to center the image horizontally
		ratio = targetHeight / svgHeight;
		if (center)
			xOffset = (targetWidth - ratio * svgWidth) / 2.0;
	}

	svg->setScale(ratio);

	QPointF pos_ = pos;
	pos_.rx() += xOffset;
	pos_.ry() += yOffset;
	svg->setPos(pos_);
}

/**
 * @brief updateSizeIcon update the svg renderer to display the correct icon for the box's size
 * @param box the pointer to the box whose icon to update
 */
void updateSizeIcon(DiagramBox *box)
{
	if (box == NULL)
		informUserAndCrash(QObject::tr("Cannot update box's size icon: box is null!"));

	OutputType oType = box->outputType();
	QGraphicsSvgItem *sizeIcon = box->sizeIcon();
	if (sizeIcon == NULL)
		informUserAndCrash(QObject::tr("Cannot update box's size icon: svg item is null!"));

	// If the output type is SCALAR, then set the icon size as scalar
	if (oType == SCALAR)
		sizeIcon->setElementId("scalar");

	// If the output is MATRIX, then compares rows and cols to display a column vector, a row vector
	// or a general matrix
	else if (oType == MATRIX) {
		int rows = box->rows();
		int cols = box->cols();

		if (rows == 1 && cols != 1)
			sizeIcon->setElementId("row");
		else if (cols == 1 && rows != 1)
			sizeIcon->setElementId("column");
		else
			sizeIcon->setElementId("matrix");
	} else if (oType == STRING ){
		sizeIcon->setElementId("string");
	} else {
		informUserAndCrash(QObject::tr("Unsupported output type when trying to update a box's icon size. "
		                      "Supported types are SCALAR and MATRIX."));
	}

}

QString timeUnitToString(const TimeUnit &unit)
{
	if (unit == HZ)
		return "Hz";

	if (unit == MS)
		return "ms";

	informUserAndCrash(QObject::tr("Unsupported time unit when trying to convert to string. Supported "
	                      "time units are: HZ and MS"));

	return ""; // should not reach here as informUserAndCrash() crashes
}

/**
 * @brief areLinked tells whether an output slot and an input slot have a link between them
 * @param oSlot the originating output slot
 * @param iSlot the target input slot
 * @return whether they are linked or not
 */
bool areLinked(OutputSlot *oSlot, InputSlot *iSlot)
{
	if (oSlot == NULL || iSlot == NULL)
		informUserAndCrash(QObject::tr("Cannot check whether two slots are linked: at least one of them is null."));

	bool areLinked = false;
	foreach (Link *link, oSlot->outputs()) {
		if (link->to() == iSlot) {
			areLinked = true;
			break;
		}
	}

	return areLinked;
}

/**
 * @brief isFull checks whether an @InputSlot with 'multiple = false' can accept the connection
 * @param iSlot the @InputSlot we check
 * @return wether it can accept a new @Link or not
 */
bool isFull(InputSlot *iSlot) {
	return (!iSlot->multiple() && iSlot->inputs().size() > 0);
}

bool fileExists(const std::string &filename)
{
	struct stat buffer;
	return (stat (filename.c_str(), &buffer) == 0);
}



QList<QString> getKheopsNodes()
{
	std::vector<std::string> nodes;
	QList<QString> ret;

	if (!ros::master::getNodes(nodes)) {
		qDebug() << "Could not get nodes";
	    } else {
		foreach (std::string node_, nodes) {
			QString node = QString::fromStdString(node_);

			if (node.startsWith("/kheops_"))
				ret.append(node);
		}
	}

	return ret;
}

/**
 * @brief snakeCaseToPretty takes a snake_case_string and pretty print it to "Snake Case String"
 * @param str the snake case string to pretty print
 * @return the pretty print string
 */
QString snakeCaseToPretty(const QString &str)
{
	QStringList words = str.split('_', QString::SkipEmptyParts);
	QStringList capitalized;

	foreach (QString s, words) {
		capitalized << s[0].toUpper() + s.mid(1).toLower();
	}

	return capitalized.join(" ");
}

QString connectivityToString(const Connectivity &conn)
{
	if (conn == ONE_TO_ALL)
		return "ONE_TO_ALL";

	if (conn == ONE_TO_ONE)
		return "ONE_TO_ONE";

	if (conn == ONE_TO_NEI)
		return "ONE_TO_NEI";

	informUserAndCrash(QObject::tr("Unsupported Connectivity when trying to convert to string. Supported "
	                      "types are ONE_TO_ALL, ONE_TO_ONE and ONE_TO_NEI"));

	return ""; // Never reached, to prevent compier warning
}


Connectivity stringToConnectivity(const QString &str)
{
	QString lower = str.toLower();

	if (lower == "one_to_one")
		return ONE_TO_ONE;

	if (lower == "one_to_all")
		return ONE_TO_ALL;

	if (lower == "one_to_nei")
		return ONE_TO_NEI;

	informUserAndCrash(QObject::tr("Unsupported string to convert to Connectivity"));

	return INVALID_CONNECTIVITY;
}

/**
 * @brief mkTopicName creates a full ROS topic name based on the script name, and the function's
 * settings that contain the topic name
 * @param scriptName
 * @param topicName
 * @return
 */
QString mkTopicName(const QString &scriptName, const QString &topicName)
{
	return sanitizeTopicName(QString("/kheops_%1/function_%2").arg(scriptName, topicName));
}

/**
 * @brief sanitizeTopicName removes forbidden characters from topic names
 * @param name
 * @return
 */
QString sanitizeTopicName(const QString &name)
{
	QString ret = name;
	ret.remove('{');
	ret.remove('}');
	ret.remove(' ');
	ret.replace('-', '_');

	return ret;
}

QColor getTypeColor(const InputType inputType)
{
	switch (inputType) {
		case SCALAR_SCALAR:
		    return Qt::white;
		break;

		case SCALAR_MATRIX:
		case MATRIX_MATRIX:
		    return QColor(255, 84, 227, 80);
		break;

		case STRING_INPUT:
		    return QColor(0, 255, 255, 80);
		break;

		default:
		    return Qt::black;
	}
}

QColor getTypeColor(const OutputType outputType)
{
	switch (outputType) {
		case SCALAR:
		    return Qt::white;
		break;

		case MATRIX:
		    return QColor(255, 84, 227, 80);
		break;

		case STRING:
		    return QColor(0, 255, 255, 80);
		break;

		default:
		    return Qt::black;
	}
}

/**
 * @brief mkFilenameFromScript sanitizes a script name to create an associated file name. The
 * pattern is CapitalCamelCase
 * @param scriptName the script name to derive the file name from
 * @return the filename
 */
QString mkFilenameFromScript(const QString &scriptName)
{
	return snakeCaseToPretty(scriptName).remove(" ");
}
