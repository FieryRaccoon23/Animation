////////////////// Splines //////////////////
template<typename CurveType>
inline CubicSpline<CurveType>::CubicSpline()
{
	this->m_CurveMatrix.resize(4, 4);
}

template<typename CurveType>
inline void CubicSpline<CurveType>::AddControlPoints(std::vector<Eigen::Vector3d>& points)
{
	if (controlPoints.size() < 4)
	{
		// error
		return;
	}

	int numberOfCurves = 1 + ((controlPoints.size() - 4) / 3);

	int pointCount = 0;
	for (int i = 0; i < numberOfCurves; ++i)
	{
		this->m_CubicCurves.push_back(std::make_shared<CurveType>());

		int p1 = pointCount;
		int p2 = ++pointCount;
		int p3 = ++pointCount;
		int p4 = ++pointCount;

		SetControlPoints(points[p1], points[p2], points[p3], points[p4]);
	}

}

template<typename CurveType>
inline void CubicSpline<CurveType>::AppendControlPoint(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3)
{
	this->m_CubicCurves.push_back(std::make_shared<CurveType>());
	SetControlPoints(*this->m_CubicCurves[this->m_CubicCurves.size() - 2]->GetControlPoint(3), p1, p2, p3);
}

template<typename CurveType>
inline void CubicSpline<CurveType>::SetControlPoints(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3, Eigen::Vector3d& p4)
{
	this->m_CubicCurves[this->m_CubicCurves.size() - 1]->SetControlPoints(p1, p2, p3, p4);
}

template<typename CurveType>
inline Eigen::Vector3d CubicSpline<CurveType>::CalculateValueAt(double parameter)
{
	if (parameter > 1.0)
	{
		parameter = 1.0;
	}
	else if (parameter < 0.0)
	{
		parameter = 0.0;
	}

	/// <Note>
	/// The following part can be added to post processing of the spline. This can speed up the code
	/// </Note>
	double splineLength = 0.0;
	// Sum all the spline distances
	for (int i = 0; i < m_CubicCurves.size(); ++i)
	{
		//const CubicCurve* curve = m_CubicCurves[i].get();
		splineLength += m_CubicCurves[i]->CurveDistance();
	}

	double lengthOfParameter = parameter * splineLength;

	/// <Note>
	/// Create a map to save and access values quickly
	/// </Note>
	double currentLength = 0.0;
	double localCurveLength = 0.0;
	int curveIndex = 0;
	for (int i = 0; i < m_CubicCurves.size(); ++i)
	{
		localCurveLength = m_CubicCurves[i]->CurveDistance();
		currentLength += localCurveLength;
		// NOTE: check using epsilon
		if (lengthOfParameter < currentLength)
		{
			currentLength -= localCurveLength;
			lengthOfParameter -= currentLength;
			break;
		}
		++curveIndex;
	}

	double localParameter = lengthOfParameter / localCurveLength;

	if (curveIndex == this->m_CubicCurves.size())
	{
		--curveIndex;
		localParameter = 1.0;
	}

	return this->m_CubicCurves[curveIndex]->CalculateValueAt(this->m_CurveMatrix, localParameter);
}