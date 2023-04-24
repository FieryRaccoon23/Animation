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

Eigen::MatrixXd Vertices;
Eigen::MatrixXi Faces;

const Eigen::RowVector3d kColorRed(255. / 255., 1. / 255., 1. / 255.);
const Eigen::RowVector3d kColorGreen(1. / 255., 255. / 255., 1. / 255.);
const Eigen::RowVector3d kColorBlue(1. / 255., 1. / 255., 255. / 255.);
const Eigen::RowVector3d kColorPink(255. / 255., 1. / 255., 255. / 255.);

Eigen::Vector3d p1(0.0, 0.0, 0.0);
Eigen::Vector3d p2(0.3, 0.5, 0.0);
Eigen::Vector3d p3(0.8, 0.3, 0.0);
Eigen::Vector3d p4(1.0, 1.0, 0.0);

Eigen::Vector3d GetPointOnSpline(double parameter)
{
    BezierSpline b(1);
    b.SetControlPoints(0, p1, p2, p3, p4);
    return b.CalculateValueAt(parameter);
}

void DisplayPoints(igl::opengl::glfw::Viewer& viewer)
{
    // Draw points
    viewer.data().BeginDrawingPoints(4);
    viewer.data().SetPoints(p1, kColorGreen);
    viewer.data().SetPoints(p2, kColorBlue);
    viewer.data().SetPoints(p3, kColorBlue);
    viewer.data().SetPoints(p4, kColorGreen);
    viewer.data().EndDrawingPoints();
    viewer.data().point_size = 10;

    // Draw lines
    int count = 20;
    viewer.data().BeginDrawingLines(count + 2);

    Eigen::Vector3d start = GetPointOnSpline(0.0);
    Eigen::Vector3d end;

    for (int i = 1; i <= count; ++i)
    {
        end = GetPointOnSpline(i/(double)(count));
        viewer.data().SetLines(start, end, kColorRed);
        start = end;
    }

    viewer.data().SetLines(p1, p2, kColorPink);
    viewer.data().SetLines(p4, p3, kColorPink);

    viewer.data().EndDrawingLines();
    viewer.data().line_width = 2;
    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    
}

int main(int argc, char *argv[])
{
    igl::opengl::glfw::Viewer viewer;
    DisplayPoints(viewer);
    //igl::readOBJ(TUTORIAL_SHARED_PATH "/Hand.obj", Vertices, Faces);
    //viewer.data().set_mesh(Vertices, Faces);
    viewer.core().camera_zoom = 0.5;
    viewer.launch();

    return 0;
}
