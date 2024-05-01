"""
for segmentation

import rhinoinside
rhinoinside.load()
"""

try:
    import Rhino.Geometry as rg
except ImportError:
    import platform
    if platform.python_implementation() == 'IronPython':
        raise

from compas.geometry import Point, Circle
from itertools import chain
import robotic_plaster_w.utilities.util as u


def intersrctionWithBoundPlanes(curves, vetical_planes):
    """
    this function is find the intersection

    return intersect_curves_id_dict, intersect_points_dict
    # plane_i : curve_id
    # plane_i : points of curve
    """
    intersect_points = []
    intersect_cuves_IDs = []
    for i, plane in enumerate(vetical_planes):
        intersect_cuves_ID = []
        curve_pts =[]
        for j, crv in enumerate(curves):
            intersect_event = rg.Intersect.Intersection.CurvePlane(crv, plane, 0.01) # intersection with refereed plane
            if intersect_event is None:
                points = rg.Point3d(0,0,0)
            else:
                points = [event.PointA for event in intersect_event]
                intersect_cuves_ID.append(j)
                curve_pts.append(points)
        intersect_cuves_IDs.append(intersect_cuves_ID)
        intersect_points.append(curve_pts)


    intersect_curves_id_dict = dict(zip(vetical_planes , intersect_cuves_IDs)) # plane_i : curve_id
    intersect_points_dict = dict(zip(vetical_planes , intersect_points)) # plane_i : points of curve


    return intersect_curves_id_dict, intersect_points_dict

def closestCurvesByPt(curves, points):
    """
    This function is to find the closest curves by reference points
    return index of curves , closest curves

    """
    distances = []
    for i in range(len(points)):
        dist_row = []
        for c in curves:
            param = c.ClosestPoint(points[i], 0.01)[1]
            points_on_curves = c.PointAt(param)
            distance = points[i].DistanceTo(points_on_curves)
            dist_row.append(distance)
        distances.append(dist_row)

    max_distance = [min(d) for d in distances]
    closest_crv_id =[]
    closest_crv =[]
    for i, d_row in enumerate(distances):
        for j, d in enumerate(d_row):
            if d == max_distance[i]:
                closest_crv_id.append(j)
                closest_crv.append(curves[j])
    return closest_crv_id, closest_crv

### main function

def getHorizontalDistribution(curves, grid_plane, index_start = 0, index_end = 0):
    """
    find the distribution of curves

    return group_curves
    """
    # find the start pt and end pt

    mid_points = []
    for curve in curves:
        curve.Domain = rg.Interval(0.0, 1.0)
        start_point = (curve.PointAt(0.0))
        end_point = (curve.PointAt(1.0))
        mid_point = (start_point+end_point)/2
        mid_points.append(mid_point)

    points = mid_points[:]
    # extend curves
    vector_start = points[0]  - points[1]
    vector_start.Unitize()
    new_start_pt = points[0] + vector_start*300

    vector_end = points[-1]  - points[-2]
    vector_end.Unitize()
    new_end_pt = points[-1] + vector_end*300

    for pt in points:
        if pt == points[0]:
            points[0] = new_start_pt
        elif pt == points[-1]:
            points[-1] = new_end_pt
    tween_curves = [rg.NurbsCurve.Create(False, 3, points)]

    # find the interscetion with grid plan
    ids, intersect_points = intersrctionWithBoundPlanes(tween_curves, grid_plane)

    # get data from dictionary
    points = intersect_points.values()
    segmented_points = u.flattern(points, iteration = 2 )

    ids = closestCurvesByPt(curves, segmented_points)[0]
    ids.sort()

    # select curves by index
    selected_curves = curves[ids[0]+index_start : ids[1]+index_end]

    return selected_curves

def getVerticalDistribution(curves, planes, type):
    """
    find the distribution of curves

    return group_curves
    """
    # get the split plane
    split_planes = boundary[:]

    # get mid plane
    mid_plane_pt = (split_planes[0].Origin +split_planes[1].Origin)/2
    mid_plane = rg.Plane(mid_plane_pt, split_planes[0].XAxis, split_planes[0].YAxis)
    three_split_planes = split_planes[:]
    three_split_planes.insert(1, mid_plane)

    # two_robot_positon
    if type == 2:

        # find the curves in boudary
        ids_dict, points_dict = segm.intersrctionWithBoundPlanes(curves, [mid_plane])
        ids  = ids_dict.values()
        group_curves  = splitDataByID(curves, ids, group_type = 2)
        to_split_planes = split_planes[:]

    # three_robot_position
    elif type == 3:

        # find the curves in boudary
        ids_dict, points_dict = segm.intersrctionWithBoundPlanes(curves, split_planes)
        ids  = ids_dict.values()
        group_curves  = splitDataByID(curves, ids, group_type = 3)
        to_split_planes = three_split_planes[:]

    # find robot postion
    average_points =[]
    for plane, curves in zip(to_split_planes, group_curves):
        all_points = []
        for curve in curves:
            curve.Domain = rg.Interval(0.0, 1.0)
            start_pt = curve.PointAt(0.0)
            end_pt = curve.PointAt(1.0)
            all_points.append(start_pt)
            all_points.append(end_pt)
        average_point = u.averagePts(all_points)
        distance_robot_pos = average_point.DistanceTo(mid_plane.ClosestPoint(average_point))
        robot_pos_limit = 950/2
        if distance_robot_pos > robot_pos_limit:
            average_point = plane.ClosestPoint(average_point)
        else:
            average_point = average_point
        average_points.append(average_point)
    # check points in robot sphere
    out_path_ID = []
    for i, (curves, pt) in enumerate(zip(group_curves, average_points)):
        # robot reach area
        robot_sphere = rg.Sphere(pt, 1000).ToBrep()
        for j, curve in enumerate(curves):
            start_pt = curve.PointAt(0.0)
            end_pt = curve.PointAt(1.0)
            bool_start = robot_sphere.IsPointInside(start_pt, 0.01, True)
            bool_end = robot_sphere.IsPointInside(end_pt, 0.01, True)
            if bool_start and bool_end is True:
                pass
            else:
                print " path_%d_%d is out of area " %(i,j)
                out_path_ID.append([i,j])
    return group_curves, average_points, out_path_ID


def closestCrvbyPt(points, curves):

    distance =[]
    cls_pts =[]
    for pts in points:
        min_dist = 1000
        min_pt = rg.Point3d(0,0,0)
        for i in range(len(curves)):
            # find the cloest point
            cls_param = attractorCrv[i].ClosestPoint(pt, 5000)[1]
            cls_pt = attractorCrv[i].PointAt(cls_param)
            dist = pt.DistanceTo(cls_pt)
            if (minDist>dist):
                minDist = dist
                min_pt = cls_pt
        cls_pts.append(min_pt)
        distance.append(minDist)
    return distance, cls_pts


def closet_points(pointslist):
    distance = []
    closet_points = []
    for i, pt in enumerate(pointslist):
        index = rs.PointArrayClosestPoint(attractors, pt)
        # find the closest points
        cls_pt = attractors[index]
        dist = pt.DistanceTo(cls_pt)
        closet_points.append(cls_pt)
        distance.append(dist)
    return distance, closet_points













################### TBC

def rhinoPtToCompt(point):
    """
    rhino point3d to compas points
    """
    compas_points = Point(point.X,point.Y,point.Z)
    return compas_points



def getSegementedByShpere(points, robot_position, raduis = 1000):
    """
    This function is to find new segemnt points by region based on robot reached raduis

    return new segemented points, index of segemented points, curves
    """
    # using compas geometry
    pass




def CountInNestedList(datas, type = "max"):
    """
    [count, index]
    """
    counts =[]
    for data in datas:
        for da in data:
            counts.append(len(da))
        print(counts)
    if type == "max":
        result = max(counts)
        result_index = counts.index(result)
    return result, result_index



# def robotPositionGrid(point,robot_reach_raduis=700, distance=1000, reference_wall=0):
#     """
#     left_wall = 0
#     center_wall = 1
#     right_wall = 2


#     To be continued...
#     """
#     grid_points =[]
#     if reference_wall == 0:
#         # left_wall
#         center_plane = rg.Plane(point, -1*rg.Vector3d.XAxis, rg.Vector3d.ZAxis)
#         robot_reach = rg.Circle(center_plane,robot_reach_raduis)
#     right_point = point + center_plane.XAxis*distance
#     left_point = point - center_plane.XAxis*distance

#     return robot_reach


