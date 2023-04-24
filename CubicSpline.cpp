#include "CubicSpline.h"

////////////////// Curves //////////////////
CubicCurve::CubicCurve()
{
	m_Parameters.resize(1, 4);
	m_ControlPoints.resize(4);
	m_CalculatedPoints.resize(4);
}

Eigen::Vector3d CubicCurve::CalculateValueAt(const Eigen::MatrixXd& curveMatrix, double parameter)
{
	Eigen::Vector3d result(0.0, 0.0, 0.0);

#pragma omp parallel for
	for (int t = 0; t < 4; ++t)
	{
		this->m_Parameters(t) = std::pow(parameter, t);
	}

	Eigen::MatrixXd matrix = std::move(this->m_Parameters * curveMatrix);

#pragma omp parallel for
	for (int t = 0; t < 4; ++t)
	{
		this->m_CalculatedPoints[t] = this->m_ControlPoints[t] * matrix(0, t);
	}

	result = this->m_CalculatedPoints[0] + this->m_CalculatedPoints[1] + this->m_CalculatedPoints[2] + this->m_CalculatedPoints[3];

	return result;
}

void CubicCurve::SetControlPoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, const Eigen::Vector3d& p4)
{
	m_ControlPoints[0] = p1;
	m_ControlPoints[1] = p2;
	m_ControlPoints[2] = p3;
	m_ControlPoints[3] = p4;
}

// Bezier Curve
double BezierCurve::CurveDistance() const
{
	double chord = (this->m_ControlPoints[3] - this->m_ControlPoints[0]).norm();
	double sumOfDistanceOfEachPoint = (this->m_ControlPoints[1] - this->m_ControlPoints[0]).norm() +
		(this->m_ControlPoints[2] - this->m_ControlPoints[1]).norm() +
		(this->m_ControlPoints[3] - this->m_ControlPoints[2]).norm();
	// Approximate arc length
	return (sumOfDistanceOfEachPoint + chord) / 2.0;
}

