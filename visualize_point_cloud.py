#!/usr/bin/env python
"""
    Visualize Velodyne VLP-16 Point Cloud
    usage:
        ./visualize_point_cloud.py <file_path>
"""
import numpy as np
import os
import sys
import vtk
from numpy import random
import cv2
from numpy import linalg
import copy
import matplotlib.pyplot as plt


class VtkPointCloud:
    def __init__(self, zMin=-1.0, zMax=1.0, maxNumPoints=1e6):
        self.init_planes()
        self.init_points(zMin, zMax, maxNumPoints)

    def addPoint(self, point, color_num):
        if self.vtkPoints.GetNumberOfPoints() < self.maxNumPoints:
            pointId = self.vtkPoints.InsertNextPoint(point[:])
            self.vtkDepth.InsertNextValue(color_num)
            self.vtkCells.InsertNextCell(1)
            self.vtkCells.InsertCellPoint(pointId)
        else:
            r = random.randint(0, self.maxNumPoints)
            self.vtkPoints.SetPoint(r, point[:])
        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()

    def addPlane(self, plane_center, normal, x_axis, y_axis):
        self.vtkPlanes.SetCenter(plane_center)
        self.vtkPlanes.SetNormal(normal)
        self.vtkPlanes.SetPoint1(x_axis)
        self.vtkPlanes.SetPoint2(y_axis)

    def init_points(self, zMin=-1.0, zMax=1.0, maxNumPoints=1e6):
        self.maxNumPoints = maxNumPoints
        self.vtkPolyData = vtk.vtkPolyData()
        self.vtkPoints = vtk.vtkPoints()
        self.vtkCells = vtk.vtkCellArray()
        self.vtkDepth = vtk.vtkDoubleArray()

        self.vtkDepth.SetName('DepthArray')
        self.vtkPolyData.SetPoints(self.vtkPoints)
        self.vtkPolyData.SetVerts(self.vtkCells)
        self.vtkPolyData.GetCellData().SetScalars(self.vtkDepth)
        self.vtkPolyData.GetCellData().SetActiveScalars('DepthArray')
        point_mapper = vtk.vtkPolyDataMapper()
        point_mapper.SetInput(self.vtkPolyData)
        point_mapper.SetColorModeToDefault()
        point_mapper.SetScalarRange(zMin, zMax)
        self.point_vtkActor = vtk.vtkActor()
        self.point_vtkActor.SetMapper(point_mapper)

    def init_planes(self):
        self.vtkPlanes = vtk.vtkPlaneSource()
        plane_mapper = vtk.vtkPolyDataMapper()
        plane_mapper.SetInput(self.vtkPlanes.GetOutput())
        self.plane_vtkActor = vtk.vtkActor()
        self.plane_vtkActor.SetMapper(plane_mapper)


def project2zplane(r_mtx, t_vec, i_mtx, z_height, img_pos):
    pro_mtx = np.dot(i_mtx, np.vstack((r_mtx.T, t_vec.T)).T)
    a = copy.deepcopy(pro_mtx[:, 0:3])
    b = np.zeros((3, 1))
    a[0, 2] = -img_pos[0]
    b[0, 0] = -(pro_mtx[0, 2] * z_height + pro_mtx[0, 3])
    a[1, 2] = -img_pos[1]
    b[1, 0] = -(pro_mtx[1, 2] * z_height + pro_mtx[1, 3])
    a[2, 2] = -1.0
    b[2, 0] = -(pro_mtx[2, 2] * z_height + pro_mtx[2, 3])
    result = linalg.solve(a, b)
    x_3d = result[0]
    y_3d = result[1]
    return x_3d, y_3d


def vtk_visualize(point_list, view_thresh):
    point_cloud = VtkPointCloud()
    x_thresh = view_thresh[0]
    y_thresh = view_thresh[1]
    z_thresh = view_thresh[2]

    for i in range(len(point_list)):
        point_coords = point_list[i]

        if (point_coords[0] > x_thresh[0]) and (point_coords[0] < x_thresh[1]) and \
                (point_coords[1] > y_thresh[0]) and (point_coords[1] < y_thresh[1]) and \
                (point_coords[2] > z_thresh[0]) and (point_coords[2] < z_thresh[1]):
            color_num = 0.7
        else:
            color_num = -1
        point_cloud.addPoint(point_list[i], color_num)

    # Add the velodyne plane
    for x in np.linspace(-4, 4, 100):
        for y in np.linspace(0, 2, 25):
            tmp_coords = np.array([x, 0, y])
            point_cloud.addPoint(tmp_coords, 1)
    # Add the floor plane
    plane_center = (-4, -4, -0.55)
    normal = (0, 0, 1)
    point1 = ([-4, 10, -0.55])
    point2 = ([4, -4, -0.55])
    point_cloud.addPlane(plane_center, normal, point1, point2)

    # Renderer
    renderer = vtk.vtkRenderer()
    renderer.AddActor(point_cloud.point_vtkActor)
    renderer.AddActor(point_cloud.plane_vtkActor)

    renderer.SetBackground(0.0, 0.0, 0.0)
    renderer.ResetCamera()

    # Render Window
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)

    # Interactor
    render_window_interactor = vtk.vtkRenderWindowInteractor()
    render_window_interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
    render_window_interactor.SetRenderWindow(render_window)

    '''Add camera coordinates'''
    axes = vtk.vtkAxesActor()
    widget = vtk.vtkOrientationMarkerWidget()
    widget.SetOutlineColor(0.9300, 0.5700, 0.1300)
    widget.SetOrientationMarker(axes)
    widget.SetInteractor(render_window_interactor)
    widget.SetViewport(0.0, 0.0, 0.4, 0.4)
    widget.SetEnabled(1)
    widget.InteractiveOn()
    render_window.Render()
    render_window_interactor.Start()


def load_data(point_cloud_path):
    # 0: left and right (-5:5); 1: near and further (10:-30); 2: up and down (-1:3)
    # 0: left < right
    # 1: near < further
    # 2: up < down
    # x_thresh = [-5, 5]
    # y_thresh = [-30, 10]
    # z_thresh = [-1, 3]

    # x_thresh = [-1.1, 1.8]
    # y_thresh = [-2.5, -1.5]
    # z_thresh = [-0.4, 2.1]

    x_thresh = [-1.1, 2.1]
    y_thresh = [1.5, 4.5]
    z_thresh = [-0.4, 2.1]

    file_path = os.path.join(point_cloud_path)
    # file_path = os.path.join(point_cloud_path, 'frame('+str(img_num)+').csv')
    point_list = []
    all_point_list = []
    with open(file_path) as f:
        f.readline()
        while True:
            data = f.readline()
            if not data:
                break
            point_coords = np.float64(data.strip().split(',')[:3])
            all_point_list.append(point_coords)
            if (point_coords[0] > x_thresh[0]) and (point_coords[0] < x_thresh[1]) and \
                    (point_coords[1] > y_thresh[0]) and (point_coords[1] < y_thresh[1]) and \
                    (point_coords[2] > z_thresh[0]) and (point_coords[2] < z_thresh[1]):
                point_list.append(point_coords)
    point_list = np.array(point_list)
    all_point_list = np.array(all_point_list)
    thresh = [x_thresh, y_thresh, z_thresh]
    return point_list, all_point_list, thresh


def main(point_cloud_path):
    data, all_data, view_thresh = load_data(point_cloud_path)
    vtk_visualize(all_data, view_thresh)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    main(sys.argv[1])
