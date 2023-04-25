#include "IMGUIInterface.h"
#include <igl/opengl/glfw/Viewer.h>

void IMGUIInterface::Init(igl::opengl::glfw::Viewer& viewer)
{
	viewer.plugins.push_back(&m_ImguiPlugin);
	m_ImguiPlugin.widgets.push_back(&m_Gizmo);

	m_Gizmo.visible = true;
	m_Gizmo.operation = ImGuizmo::TRANSLATE;

    viewer.callback_key_pressed = [&](decltype(viewer)&, unsigned int key, int mod)
    {
        return this->KeyDownCallback(viewer, key, mod);
    };

    m_Gizmo.callback = [&](const Eigen::Matrix4f& T)
    {
        GizmoTransformCallback(T);
    };
}

void IMGUIInterface::InitSplineData(const Eigen::Vector3d* p1, const Eigen::Vector3d* p2, const Eigen::Vector3d* p3, const Eigen::Vector3d* p4)
{
    m_SplineControlPoints.push_back(p1);
    m_SplineControlPoints.push_back(p2);
    m_SplineControlPoints.push_back(p3);
    m_SplineControlPoints.push_back(p4);

    SetTransformToPoint(m_Gizmo.T, p1);

    m_InitialTransform = Eigen::Matrix4f::Identity();
    SetTransformToPoint(m_InitialTransform, p1);
}

void IMGUIInterface::UpdateGizmoToSelectedPoint()
{
    SetTransformToPoint(m_Gizmo.T, m_SplineControlPoints[m_SelectedControlPoint]);
}

bool IMGUIInterface::KeyDownCallback(igl::opengl::glfw::Viewer& viewer, unsigned char key, int mods)
{
    switch (key)
    {
    case 'N':
    case 'n':
        m_SelectedControlPoint = m_SelectedControlPoint == 3 ? 0 : ++m_SelectedControlPoint;
        UpdateGizmoToSelectedPoint();
        return true;
    case 'P':
    case 'p':
        m_SelectedControlPoint = m_SelectedControlPoint == 0 ? 3 : --m_SelectedControlPoint;
        UpdateGizmoToSelectedPoint();
        return true;
    }
    return false;
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

void IMGUIInterface::SetTransformToPoint(Eigen::Matrix4f& T, const Eigen::Vector3d* point)
{
    T(0, 3) = point->x();
    T(1, 3) = point->y();
    T(2, 3) = point->z();
}

const Eigen::Vector3d IMGUIInterface::GetGizmoPosition() const
{
    return Eigen::Vector3d(m_Gizmo.T(0, 3), m_Gizmo.T(1, 3), m_Gizmo.T(2, 3));
}