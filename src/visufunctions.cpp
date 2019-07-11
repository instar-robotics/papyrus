#include "visufunctions.h"

VisuType stringToVisuType(const QString &str)
{
	QString visuType = str.toUpper();
	if(visuType == "THERMAL 2D")
		return THERMAL_2D;
	if(visuType == "SURFACE 3D")
		return SURFACE_3D;
	if(visuType == "BAR CHART 3D")
		return BAR_CHART_3D;
	if(visuType == "CONE CHART 3D")
		return CONE_CHART_3D;
	if(visuType == "WIREFRAME 3D")
		return WIREFRAME_3D;
	if(visuType == "CROWN 3D")
		return CROWN_3D;
	if(visuType == "BAR CIRCLE 3D")
		return BAR_CIRCLE_3D;
	if(visuType == "WIREFRAME POLAR 3D")
		return WIREFRAME_POLAR_3D;
	if(visuType == "BAR POLAR 3D")
		return BAR_POLAR_3D;
	if(visuType == "SURFACE POLAR 3D")
		return SURFACE_POLAR_3D;
	return UNKNOWN;
}

QString visuTypeToString(const VisuType &visuType)
{
	if(visuType == THERMAL_2D)
		return "Thermal 2D";
	if(visuType == SURFACE_3D)
		return "Surface 3D";
	if(visuType == BAR_CHART_3D)
		return "Bar chart 3D";
	if(visuType == CONE_CHART_3D)
		return "Cone chart 3D";
	if(visuType == WIREFRAME_3D)
		return "Wireframe 3D";
	if(visuType == CROWN_3D)
		return "Crown 3D";
	if(visuType == BAR_CIRCLE_3D)
		return "Bar circle 3D";
	if(visuType == WIREFRAME_POLAR_3D)
		return "Wireframe polar 3D";
	if(visuType == BAR_POLAR_3D)
		return "Bar polar 3D";
	if(visuType == SURFACE_POLAR_3D)
		return "Surface polar 3D";
	return "Unknown";
}

bool is2DVisuType(const VisuType &visuType)
{
	if(visuType == THERMAL_2D)
		return true;
	return false;
}

bool is3DVisuType(const VisuType &visuType)
{
	if(visuType == SURFACE_3D ||
	   visuType == BAR_CHART_3D ||
	   visuType == CONE_CHART_3D ||
	   visuType == WIREFRAME_3D ||
	   visuType == CROWN_3D ||
	   visuType == BAR_CIRCLE_3D ||
	   visuType == WIREFRAME_POLAR_3D ||
	   visuType == BAR_POLAR_3D ||
	   visuType == SURFACE_POLAR_3D)
		return true;
	return false;
}

bool is3DMatrixVisuType(const VisuType &visuType)
{
	if(visuType == SURFACE_3D ||
	   visuType == BAR_CHART_3D ||
	   visuType == CONE_CHART_3D ||
	   visuType == WIREFRAME_3D)
		return true;
	return false;
}

bool is3DCircularVisuType(const VisuType &visuType)
{
	if(visuType == CROWN_3D ||
	   visuType == BAR_CIRCLE_3D)
		return true;
	return false;
}

bool is3DPolarVisuType(const VisuType &visuType)
{
	if(visuType == WIREFRAME_POLAR_3D ||
	   visuType == BAR_POLAR_3D ||
	   visuType == SURFACE_POLAR_3D)
		return true;
	return false;
}

ShaderWidget* createShaderWidget(VisuType type, int rows, int cols, QVector<QVariant> parameters)
{
	if(is3DMatrixVisuType(type))
	{
		if(type== BAR_CHART_3D)
			return new ShaderBarChart(rows, cols);
		else if(type == CONE_CHART_3D)
			return new ShaderConeChart(rows, cols);
		else if(type == WIREFRAME_3D)
			return new ShaderWireframe(rows, cols);
		else
			return new ShaderSurface(rows, cols);
	}
	else if(is3DCircularVisuType(type))
	{
		if(rows != 1 && cols != 1)
			return new ShaderSurface(rows, cols);
		else if(type == BAR_CIRCLE_3D)
		{
			if(rows == 1)
				return new ShaderBarCircle(cols, cols/2, CLOCKWISE);
			else
				return new ShaderBarCircle(rows, rows/2, CLOCKWISE);
		}
		else
		{
			if(rows == 1)
				return new ShaderCrown(cols, cols/2, CLOCKWISE);
			else
				return new ShaderCrown(rows, rows/2, CLOCKWISE);
		}
	}
	else if(is3DPolarVisuType(type))
	{
		int indexZero = cols/2;
		RotationDir rotationDir = CLOCKWISE;
		if(parameters.size() == 2)
		{
			rotationDir = RotationDir(parameters.at(0).toInt());
			indexZero = parameters.at(1).toInt();
		}
		if(type == BAR_POLAR_3D)
			return new ShaderBarPolar(rows, cols, indexZero, rotationDir);
		else if(type == WIREFRAME_POLAR_3D)
			return new ShaderWireframePolar(rows, cols, indexZero, rotationDir);
		else
			return new ShaderSurfacePolar(rows, cols, indexZero, rotationDir);
	}
	else
		return new ShaderSurface(rows, cols);
}

RotationDir intToRotationDir(int rotDir)
{
	if(rotDir <= 0)
		return CLOCKWISE;
	else
		return COUNTERCLOCKWISE;
}

void copyVisuParameters(QVector<QVariant> origin, QVector<QVariant> *destination)
{
	destination->clear();
	for(int i = 0; i<origin.size(); i++)
	{
		destination->push_back(origin.at(i));
	}
}
