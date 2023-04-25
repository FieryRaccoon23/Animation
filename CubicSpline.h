#pragma once

#include <Eigen/Geometry>
#include <vector>

////////////////// Curves //////////////////
class CubicCurve
{
public:
	CubicCurve();
	virtual ~CubicCurve() = default;

	Eigen::Vector3d  CalculateValueAt(const Eigen::MatrixXd& curveMatrix, double parameter);
	void		     SetControlPoints(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3, Eigen::Vector3d& p4);
	Eigen::Vector3d*  GetControlPoint(int i) const;
	
	virtual double   CurveDistance() const { return 0.0; }
	
protected:
	Eigen::MatrixXd				    m_Parameters;
	std::vector<Eigen::Vector3d*>	m_ControlPoints;
	std::vector<Eigen::Vector3d>    m_CalculatedPoints;
};

// Bezier Curve
class BezierCurve final: public CubicCurve
{
public:
	double CurveDistance() const override;
};


////////////////// Splines //////////////////
template<typename CurveType>
class CubicSpline
{
public:
	CubicSpline();
	virtual ~CubicSpline() = default;

	Eigen::Vector3d CalculateValueAt(double parameter);
	void			AddControlPoints(std::vector<Eigen::Vector3d>& points);
	void			AppendControlPoint(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3);
	void		    SetControlPoints(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3, Eigen::Vector3d& p4);

protected:
	std::vector<std::shared_ptr<CurveType>> m_CubicCurves;
	Eigen::MatrixXd m_CurveMatrix;
};

// Bezier Spline
class BezierSpline final : public CubicSpline<BezierCurve>
{
public:
	BezierSpline() : CubicSpline<BezierCurve>()
	{
		this->m_CurveMatrix <<  1.0, 0.0, 0.0, 0.0,
							   -3.0, 3.0, 0.0, 0.0, 
							    3.0, -6.0, 3.0, 0.0, 
							   -1.0, 3.0, -3.0, 1.0;
	}

};

// Basis Spline
class BasisSpline final : public CubicSpline<BezierCurve>
{
public:
	BasisSpline() : CubicSpline<BezierCurve>()
	{
		this->m_CurveMatrix << 1.0, 4.0, 1.0, 0.0,
							  -3.0, 0.0, 3.0, 0.0,
			                   3.0, -6.0, 3.0, 0.0,
			                  -1.0, 3.0, -3.0, 1.0;
	}

};

#include "CubicSpline.inl"