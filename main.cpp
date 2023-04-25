#include <igl/directed_edge_orientations.h>
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
#include <igl/PI.h>
#include <igl/lbs_matrix.h>
#include <igl/deform_skeleton.h>
#include <igl/dqs.h>
#include <igl/readDMAT.h>
#include <igl/readOBJ.h>
#include <igl/readTGF.h>
#include <igl/opengl/glfw/Viewer.h>

#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>

#include "CubicSpline.h"
#include "IMGUIInterface.h"

Eigen::MatrixXd Vertices;
Eigen::MatrixXi Faces;

const Eigen::RowVector3d kColorRed(255. / 255., 1. / 255., 1. / 255.);
const Eigen::RowVector3d kColorGreen(1. / 255., 255. / 255., 1. / 255.);
const Eigen::RowVector3d kColorBlue(1. / 255., 1. / 255., 255. / 255.);
const Eigen::RowVector3d kColorPink(255. / 255., 1. / 255., 255. / 255.);

std::vector<Eigen::Vector3d> controlPoints;
BezierSpline bezierSpline;
igl::opengl::glfw::Viewer viewer;
IMGUIInterface imguiInterface;

void InitControlPoints()
{
    controlPoints.push_back(Eigen::Vector3d(0.0, 5.0, 0.0));
    controlPoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
    controlPoints.push_back(Eigen::Vector3d(9.0, 0.0, 0.0));
    controlPoints.push_back(Eigen::Vector3d(10.0, 5.0, 0.0));
    controlPoints.push_back(Eigen::Vector3d(11.0, 10.0, 0.0));
    controlPoints.push_back(Eigen::Vector3d(19.0, 10.0, 0.0));
    controlPoints.push_back(Eigen::Vector3d(20.0, 5.0, 0.0));
}

void CreateSpline()
{
    bezierSpline.AddControlPoints(controlPoints);
}

void AddCurveToSpline()
{
    bezierSpline.AppendControlPoint(controlPoints[4], controlPoints[5], controlPoints[6]);
}

Eigen::Vector3d GetPointOnSpline(double parameter)
{
    return bezierSpline.CalculateValueAt(parameter);
}

void DisplaySpline()
{
    // Draw points
    viewer.data().BeginDrawingPoints(controlPoints.size());
    for (int i = 0; i < controlPoints.size(); ++i)
    {
        viewer.data().SetPoints(controlPoints[i], kColorGreen);
    }
    viewer.data().EndDrawingPoints();
    viewer.data().point_size = 10;

    // Draw lines
    int count = 20;
    viewer.data().BeginDrawingLines(count /*+ 2*/);

    Eigen::Vector3d start = GetPointOnSpline(0.0);
    Eigen::Vector3d end;

    for (int i = 1; i <= count; ++i)
    {
        end = GetPointOnSpline(i/(double)(count));
        viewer.data().SetLines(start, end, kColorRed);
        start = end;
    }

    //viewer.data().SetLines(p1, p2, kColorPink);
    //viewer.data().SetLines(p4, p3, kColorPink);

    viewer.data().EndDrawingLines();
    viewer.data().line_width = 2;
    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    
}

void UpdateControlPoint(GizmoTransformCallbackPayload& payload)
{
    Eigen::Vector3d position = Eigen::Vector3d(payload.m_Transform(0, 3), payload.m_Transform(1, 3), payload.m_Transform(2, 3));
    int selectedControlPointIndex = payload.m_SelectedControlPoint;

    Eigen::Vector3d* selectedControlPoint = &controlPoints[selectedControlPointIndex];

    *selectedControlPoint = position;

    DisplaySpline();
}

bool KeyDownCallback(igl::opengl::glfw::Viewer& viewer, unsigned char key, int mods)
{
    switch (key)
    {
    case 'N':
    case 'n':
        imguiInterface.IncrementSelectedControlPointIndex();
        return true;
    case 'P':
    case 'p':
        imguiInterface.DecrementSelectedControlPointIndex();
        return true;
    }
    return false;
}

//bool PreDrawCallback(igl::opengl::glfw::Viewer& viewer)
//{
//
//    return false;
//}

int main(int argc, char *argv[])
{
    InitControlPoints();
    CreateSpline();
    //AddCurveToSpline();
    DisplaySpline();

    //IMGUIInterface imguiInterface;
    imguiInterface.Init(viewer);
    imguiInterface.InitSplineData(controlPoints);
    imguiInterface.m_GizmoTransformCallback = &UpdateControlPoint;

    //igl::readOBJ(TUTORIAL_SHARED_PATH "/Hand.obj", Vertices, Faces);
    //viewer.data().set_mesh(Vertices, Faces);
    //viewer.callback_pre_draw = &PreDrawCallback;
    viewer.callback_key_down = &KeyDownCallback;
    viewer.core().camera_zoom = 0.5;
    viewer.launch();

    return 0;
}
