////////////////// Splines //////////////////
template<typename CurveType>
inline CubicSpline<CurveType>::CubicSpline(int numberOfCurves)
{
	for (int i = 0; i < numberOfCurves; ++i)
	{
		this->m_CubicCurves.push_back(std::make_shared<CurveType>());
	}
	this->m_CurveMatrix.resize(4, 4);
}

template<typename CurveType>
void CubicSpline<CurveType>::SetControlPoints(int index, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, const Eigen::Vector3d& p4)
{
	if (index >= m_CubicCurves.size())
	{
		// error
		return;
	}

	m_CubicCurves[index]->SetControlPoints(p1, p2, p3, p4);
}

template<typename CurveType>
Eigen::Vector3d CubicSpline<CurveType>::CalculateValueAt(double parameter)
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

	double actualLength = parameter * splineLength;

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
		if (actualLength <= currentLength)
		{
			break;
		}
		++curveIndex;
	}

	double localParameter = actualLength / localCurveLength;

	return this->m_CubicCurves[curveIndex]->CalculateValueAt(this->m_CurveMatrix, localParameter);
}