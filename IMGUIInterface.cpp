#include "IMGUIInterface.h"
#include <igl/opengl/glfw/Viewer.h>

void IMGUIInterface::Init(igl::opengl::glfw::Viewer& viewer)
{
	viewer.plugins.push_back(&m_ImguiPlugin);
	m_ImguiPlugin.widgets.push_back(&m_Gizmo);

	m_Gizmo.visible = true;
	m_Gizmo.operation = ImGuizmo::TRANSLATE;

    m_Gizmo.callback = [&](const Eigen::Matrix4f& T)
    {
        GizmoTransformCallback(T);
    };
}

void IMGUIInterface::InitSplineData(const std::vector<Eigen::Vector3d>& controlPoints)
{
    m_SplineControlPoints = &controlPoints;

    SetTransformToPoint(m_Gizmo.T, m_SplineControlPoints->at(0));
}

void IMGUIInterface::UpdateGizmoToSelectedPoint()
{
    SetTransformToPoint(m_Gizmo.T, m_SplineControlPoints->at(m_SelectedControlPoint));
}

void IMGUIInterface::IncrementSelectedControlPointIndex()
{
    m_SelectedControlPoint = m_SelectedControlPoint == m_SplineControlPoints->size() - 1 ? 0 : ++m_SelectedControlPoint;
    UpdateGizmoToSelectedPoint();
}

void IMGUIInterface::DecrementSelectedControlPointIndex()
{
    m_SelectedControlPoint = m_SelectedControlPoint == 0 ? m_SplineControlPoints->size() - 1 : --m_SelectedControlPoint;
    UpdateGizmoToSelectedPoint();
}

void IMGUIInterface::GizmoTransformCallback(const Eigen::Matrix4f& T)
{
    if (m_GizmoTransformCallback)
    {
        GizmoTransformCallbackPayload payload;
        payload.m_Transform = T;
        payload.m_SelectedControlPoint = m_SelectedControlPoint;
        m_GizmoTransformCallback(payload);
    }
}

void IMGUIInterface::SetTransformToPoint(Eigen::Matrix4f& T, const Eigen::Vector3d& point)
{
    T(0, 3) = point.x();
    T(1, 3) = point.y();
    T(2, 3) = point.z();
}

const Eigen::Vector3d IMGUIInterface::GetGizmoPosition() const
{
    return Eigen::Vector3d(m_Gizmo.T(0, 3), m_Gizmo.T(1, 3), m_Gizmo.T(2, 3));
}