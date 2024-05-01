
"""
This libraries is for MAS T2 , robotic plaster spraring .
based on rhino geometry

import rhinoinside
rhinoinside.load()
"""
try:
    import Rhino.Geometry as rg
except ImportError:
    import platform
    if platform.python_implementation() == 'IronPython':
        raise

from copy import copy, deepcopy
from itertools import chain
from compas.geometry import Point

# __all__ = [ ]


### categories
# for datastructure
# for geometry
## curves ; transform; points


#####
# datastructure
def flattern(data, iteration = 1):
    """
    this function is to flattern the data

    return list

    it could be
    data = list(chain(*data))
    data = list(chain.from_iterable(data))
    """
    for _ in range(iteration):
        data = sum(data, [])

    return data

def flatternInside(data, iteration =1):
    """
    this function is to flattern the data from inside

    return list

    """
    new_data =[]
    for da in data:
        if isinstance(da, list):
            new_data.append(sum(da,[]))
        else:
            new_data = list(data)
    return new_data


def flipMatrix(data):
    """
    This function is flip matrix data
    example:
        list = [[1,2,3],[4,5,6],[7,8,9]]
        fplist  = [[1,4,7], [2,5,8], [3,6,9]]
    retrun list
    """
    fpList = []
    number_item = len(data[0])
    for i in range(number_item):
        fpList.append([row[i] for row in data])

    return fpList

def remapValue(value, input_min, input_max, target_min, target_max):
    remap = ((value-input_min)/(input_max-input_min))*(target_max-target_min)+target_min
    return remap

def divideDataByindex(data, index):
    """
    return group data
    """
    group_data = []
    for i in range(len(index)-1):
        if i == 0:
            group_data.append([data[index[i]:index[i+1]]])
        else:
            group_data.append([data[index[i]:index[i+1]+1]])
    return group_data


#####
# geometry
def sortedCurveByMidpt(curves, direction = 0, reverse_list = True):
    """
    This function is to sort curves by which direction
    direction
    X : 0
    Y : 1
    Z : 2
    retrun curves
    """
    # define direction

    for curve in curves:
        domain = rg.Interval(0.0,1.0)
        curve.Domain = domain

    if direction is 0:
        sorted_curves = sorted(curves, key= lambda c :c.PointAt(0.5).X, reverse= reverse_list)
    if direction is 1:
        sorted_curves = sorted(curves, key= lambda c :c.PointAt(0.5).Y, reverse= reverse_list)
    if direction is 2:
        sorted_curves = sorted(curves, key= lambda c :c.PointAt(0.5).Z, reverse= reverse_list)

    return sorted_curves

def removeDupliacteCurves(curves):
    """
    This function is to remove duplicated curves by mid points

    return curves
    """
    clearned_curves =[]

    mid_pts=[curve.PointAt(0.5) for curve in curves]

    removed_pts =[]
    for i, mid_pt in enumerate(mid_pts):
        if mid_pt not in removed_pts:
            removed_pts.append(mid_pt)
            clearned_curves.append(curves[i])
    return clearned_curves


def getDividePoints(curves, type = "dvc", divide_param = 10):
    """
    input curves should be list
    return points

    type: dvc is divide by count; dvl is divide by length

    """
    points_list = []
    if type == "dvc":
        for c in curves:
            params = c.DivideByCount(divide_param, True)
            points = [c.PointAt(p) for p in params]
            points_list.append(points)

    if type == "dvl":
        for c in curves:
            params = c.DivideByLength(divide_param, True)
            points = [c.PointAt(p) for p in params]
            points_list.append(points)

    return points_list


def getStEdPoints(curves):
    """
    get the curves start points and end points
    return [startpt, endpt]

    """
    all_points = []
    for c in curves:
        domain = rg.Interval(0.0, 1.0)
        c.Domain = domain
        start_points = (c.PointAt(0.0))
        end_points = (c.PointAt(1.0))
        all_points.append(zip(start_points, end_points))

    return all_points


def averagePts(points):
    """
    find the average point of points

    return average point
    """
    new_points = points[:]
    count = len(new_points)

    pt_X = sum([pt.X for pt in new_points])/count
    pt_Y = sum([pt.Y for pt in new_points])/count
    pt_Z = sum([pt.Z for pt in new_points])/count

    average_pt = rg.Point3d(pt_X, pt_Y, pt_Z)
    return average_pt


### transformation

def projectionToPlane(pointslist, referencePlane):
    """
    return pro
    """
    projected_points = []
    transform = rg.Transform.PlanarProjection(referencePlane)
    for point in pointslist:
        point.Transform(transform)
        projected_points.append(point)
    return projected_points

def scale(points, ref_plane, factor):
    # reference plane
    transformed_points = []
    transform = rg.Transform.Scale(ref_plane, factor, factor, factor)

    new_points = deepcopy(points)

    for point in new_points:
        scale_point = point
        scale_point.Transform(transform)
        transformed_points.append(scale_point)
    return transformed_points

def getPointSelection(proportionStart, proportionEnd, pointslist):

    indexStart = int(proportionStart)
    indexEnd = int(proportionEnd)
    if proportionStart >= len(pointslist):
        indexStart = len(pointslist)
    if proportionEnd >= len(pointslist):
        indexEnd = len(pointslist)

    selectedPointSet = pointslist[indexStart: indexEnd]
    return selectedPointSet

def makeCurveFromPoints(points):
    try:
        newCurve = rg.NurbsCurve.CreateInterpolatedCurve(points, 3)
        return newCurve
    except:
        print ("error making interpolated curve")
    return newCurve

def changePlaneOrigin(planes, refPoints):
    new_planes = []
    for point in refPoints:
        for plane in planes:
            new_plane = rg.Plane(point, plane.XAxis, plane.YAxis)
            new_planes.append(new_plane)
    return new_planes

def reduceControlPoints(pointslist, tolerance=0.01):
    """
    return simpified_crv, simpified_points
    """
    curves = rg.Polyline(pointslist)
    new_curves = curves.Duplicate()
    # simpify control points of polycurve
    new_curves.ReduceSegments(tolerance)
    new_curves.Smooth(tolerance)

    simpified_points = new_curves
    simpified_curve = rg.NurbsCurve.Create(False, 3, simpified_points)
    return simpified_points, simpified_curve

def sortData(data, keys):
    """
    sort points by points coordinates,  0 : X , 1 : Y , 2 : Z
    return original data stucture
    """

    del_duop_data = set(data)
    if keys == 0:
        new_data = sorted(del_duop_data, key = lambda d : d.X)
    elif keys == 1:
        new_data = sorted(del_duop_data, key = lambda d : d.Y)
    else:
        new_data = sorted(del_duop_data, key = lambda d : d.Z)
    return new_data

def getDataSelection(proportionStart, proportionEnd, data):
    # remap_id
    data_id = [ i for i in range(len(data))]
    indexStart = int(remapValue(proportionStart, 0, 100, 0, len(data_id)))
    indexEnd= int(remapValue(proportionEnd, 0, 100, 0, len(data_id)))

    selectedDataSet = data[indexStart: indexEnd]
    selectedDataSet_id = [i+indexStart for i, p in enumerate(selectedDataSet)]

    return selectedDataSet , selectedDataSet_id

### rhino - compas

### python structure

def Covert(listData,listData_02):
    """
    covert list to dictionary
    """
    it = iter(listData)
    it2 = iter(listData_02)
    result = dict(zip(listData, listData_02))
    return result

