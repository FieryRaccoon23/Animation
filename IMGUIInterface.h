#pragma once

#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuizmoWidget.h>

struct GizmoTransformCallbackPayload
{
	Eigen::Matrix4f m_Transform;
	int m_SelectedControlPoint;
};

class IMGUIInterface
{
public:
	void Init(igl::opengl::glfw::Viewer& viewer);
	void InitSplineData(const std::vector<Eigen::Vector3d>& controlPoints);
	const Eigen::Vector3d GetGizmoPosition() const;
	void IncrementSelectedControlPointIndex();
	void DecrementSelectedControlPointIndex();

	std::function<void(GizmoTransformCallbackPayload&)> m_GizmoTransformCallback;

private:
	void GizmoTransformCallback(const Eigen::Matrix4f& T);
	void SetTransformToPoint(Eigen::Matrix4f& T, const Eigen::Vector3d& point);
	void UpdateGizmoToSelectedPoint();

	igl::opengl::glfw::imgui::ImGuiPlugin m_ImguiPlugin;
	igl::opengl::glfw::imgui::ImGuizmoWidget m_Gizmo;
	const std::vector<Eigen::Vector3d>* m_SplineControlPoints;
	int m_SelectedControlPoint = 0;
};