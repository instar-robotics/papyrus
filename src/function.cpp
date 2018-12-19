#include "function.h"

Function::Function(const QString &path) : QTreeWidgetItem(),
                                          m_descriptionFile(path),
                                          m_constant(false),
                                          m_matrixShape(SHAPE_NONE)
{

}

/**
 * @brief Function::updateTooltip updates the tooltip with the function description and
 * all its slots's descrition
 */
void Function::updateTooltip()
{
	QString desc = "<h3>" + m_name + "</h3>";
	desc += "<p>" + m_description + "</p>";
	desc += "<p>Inputs:</p>";
	desc += "<ul>";
	foreach (InputSlot *input, m_inputs) {
		desc += "<li><strong>";
		desc += input->name() + ": </strong>";
		desc += input->description();
		desc += "</li>";
	}
	setToolTip(0, desc);
}

QString Function::name() const
{
	return m_name;
}

void Function::setName(const QString &name)
{
	m_name = name;
}

std::vector<InputSlot *> Function::inputs() const
{
	return m_inputs;
}

void Function::addInputSlot(InputSlot *slot)
{
	m_inputs.push_back(slot);
}

OutputSlot* Function::output() const
{
	return m_output;
}

void Function::setOutput(OutputSlot *output)
{
	m_output = output;
}

QString Function::descriptionFile() const
{
	return m_descriptionFile;
}

bool Function::constant() const
{
	return m_constant;
}

void Function::setConstant(bool constant)
{
	m_constant = constant;
}

QString Function::libName() const
{
	return m_libName;
}

void Function::setLibName(const QString &libName)
{
	m_libName = libName;
}

QString Function::iconFilepath() const
{
	return m_iconFilepath;
}

void Function::setIconFilepath(const QString &value)
{
	m_iconFilepath = value;
}

QString Function::description() const
{
	return m_description;
}

void Function::setDescription(const QString &description)
{
	m_description = description;
}

MatrixShape Function::matrixShape() const
{
	return m_matrixShape;
}

void Function::setMatrixShape(const MatrixShape &matrixShape)
{
	m_matrixShape = matrixShape;
}
