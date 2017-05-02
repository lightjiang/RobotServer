#!/usr/bin/env python

# Copyright 2016 Fetch Robotics Inc.
# Author: Michael Hwang

## @file helper_methods.py Useful methods for various components in
#  fetchcore_client

# Standard Library
from datetime import datetime
import hashlib
import math

# Third Party
import rospy
import tf
from geometry_msgs.msg import (Point,
                               Pose,
                               PoseWithCovarianceStamped,
                               PoseStamped,
                               Quaternion)

# Fetch
from fetchcore_client.settings import TIMESTAMP_FORMAT
from fetch_move_base_msgs.msg import Lanes, LaneNode, LaneEdge

def get_timestamp(dt=None):
    if dt:
        return dt.strftime(TIMESTAMP_FORMAT)
    else:
        return datetime.utcnow().strftime(TIMESTAMP_FORMAT)

## Get a datetime object for a timestamp string
def parse_timestamp(datetime_string):
    return datetime.strptime(datetime_string, TIMESTAMP_FORMAT)

## @param xyz The XYZ array output by TF transformations
## @param quat The quaternion array output by TF transformations
def strpose_from_xyz_quat(xyz, quat):
    pose = Pose(Point(*xyz), Quaternion(*quat))
    strpose = pose_to_string(pose)
    return strpose

## @brief Get the 2D heading from a geometry_msgs/Pose
def get_theta(pose):
    _, _, theta = tf.transformations.euler_from_quaternion(
         [pose.orientation.x, pose.orientation.y,
          pose.orientation.z, pose.orientation.w]
         )
    return theta

## @brief Get the 2D heading as a quaternion
def get_quaternion(theta):
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    return Quaternion(*quat)

## @brief Convert a Pose to a str with form 'x,y,theta'
def pose_to_string(pose):
    theta = get_theta(pose)
    pstring = '%.2f,%.2f,%.4f' % (pose.position.x,
                                  pose.position.y,
                                  theta)
    return pstring

## @brief Convert x, y, and theta values into a Pose object
def xytheta_to_pose(x, y, theta):
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    conv_pose = Pose(Point(x, y, 0), Quaternion(*quat))
    return conv_pose

## @brief Convert x,y,theta string to Pose object
def string_to_pose(pose_string):
    x, y, theta = [float(f) for f in pose_string.split(',')]
    return xytheta_to_pose(x, y, theta)

## @brief Convert x,y,theta string to PoseStamped object
def string_to_posestamped(pose_string, seq_num, frame='map'):
    pstamped = PoseStamped()
    pstamped.pose = string_to_pose(pose_string)
    pstamped.header.stamp = rospy.Time.now()
    pstamped.header.frame_id = frame
    pstamped.header.seq = seq_num
    return pstamped

## @brief Convert pose string to PoseWithCovarianceStamped for localization
def string_to_pcvs(pose_string):
    pcvs = PoseWithCovarianceStamped()
    pcvs.header.seq = 0
    pcvs.header.stamp = rospy.get_rostime()
    # default RVIZ uncertainty
    pcvs.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 2.5*0.06853891945200942]
    pcvs.pose.pose = string_to_pose(pose_string)
    return pcvs

## Recursive setattr() using Python built-in functions.
#  See http://stackoverflow.com/a/31174427 for original implementation.
def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)

## Recursive getattr() using Python built-in functions.
#  See http://stackoverflow.com/a/31174427 for original implementation.
def rgetattr(obj, attr):
    return reduce(getattr, [obj]+attr.split('.'))

## Method that allows evaluation of "if obj.name" without throwing an
#  AttributeError
## @returns True if **name** is both a field in obj and evaluates to True
## @returns False otherwise
def attr_in_object(obj, name):
    try:
        attr = getattr(obj, name)
        return bool(attr)
    except AttributeError:
        return False

## @brief Generate an md5 hash, for file integrity checking
## @param filename The file to get the checksum for
## @param blocksize The amount of memory to allocate at a time for reading the
#   file into the md5 function. Any power of two should suffice, but 16 kb
#   was recommended when this function was initially implemented.
## @returns An md5 hash in base64
def get_b64_md5(filename, blocksize=65536):
    fileobj = open(filename, 'rb')
    buf = fileobj.read(blocksize)
    md5 = hashlib.md5()
    while len(buf) > 0:
        md5.update(buf)
        buf = fileobj.read(blocksize)
    return md5.digest().encode('base64')[:-1]

## Convert lane annotations to a fetchcore_msgs/Graph message
## @param annotations The annotations dict from the map info. Requires
#  "nodes" and "edges" as fields.
def convert_lane_annotations_to_lanes(annotations):
    anno_nodes = annotations['nodes']
    anno_edges = annotations['edges']

    # construct empty Graph
    lanes = Lanes()
    lanes.header.frame_id = "map"  # TODO: Can this be parametrized?
    lanes.header.stamp = rospy.Time.now()

    # populate points and UUID-to-ID map
    for idx, node in enumerate(anno_nodes):
        new_node = LaneNode()
        # assign uuid
        new_node.uuid = node['uuid']
        # assign points
        new_node.point.x = node['point']['x']
        new_node.point.y = node['point']['y']
        # store generated node in graph
        lanes.nodes.append(new_node)

    # process and add edges, bidirectionally
    for edge in anno_edges:
        new_edge = LaneEdge()
        # assign uuids
        new_edge.uuid = edge['uuid']
        new_edge.source_uuid = edge['source_uuid']
        new_edge.target_uuid = edge['target_uuid']
        # store generated node in graph
        lanes.edges.append(new_edge)

    return lanes

## Calculates the bounding box for a shape defined by three points.
## @param p1 Point2D representing a corner of a shape
## @param p2 Point2D representing a corner of a shape
## @param p3 Point2D representing a corner of a shape
## @returns The bounding box's width, height, and origin for a given shape.
def get_bounding_box_for_three_point_shape(p1, p2, p3):
    xlist, ylist = zip(*[(p.x, p.y) for p in [p1, p2, p3]])

    max_x = max(xlist)
    min_x = min(xlist)
    width = abs(max_x - min_x)

    max_y = max(ylist)
    min_y = min(ylist)
    height = abs(max_y - min_y)

    origin = (min_x + width/2, min_y + height/2)

    return width, height, origin

## Calculates the bounding box for a capsule defined by a radius and two points
## @param p1 Point2D representing the endpoint of a capsule.
## @param p2 Point2D representing the endpoint of a capsule.
## @param radius float corresponding to the radius of the capsule's ends
## @returns The bounding box's width, height, and origin for the capsule.
def get_bounding_box_for_capsule(p1, p2, radius):
    max_x = max(p1.x, p2.x)
    min_x = min(p1.x, p2.x)
    max_y = max(p1.y, p2.y)
    min_y = min(p1.y, p2.y)
    height = abs(max_y - min_y) + (2 * radius)
    width = abs(max_x - min_x) + (2 * radius)
    origin = ((max_x - min_x)/2, (max_y - min_y)/2)

    return width, height, origin

## Calculates the bounding box for a circle defined by a point and radius.
## @param p1 Point2D representing the center of a circle.
## @param radius float corresponding to the radius of a circle.
def get_bounding_box_for_circle(p1, radius):
    width = height = 2 * radius
    return width, height

## @returns tuple of minimum and maximum corner points for given component
def get_bounds_for_component(component):
    p1 = component.get("p1", {"x": 0, "y": 0})
    p2 = component.get("p2", {"x": 0, "y": 0})
    p3 = component.get("p3", {"x": 0, "y": 0})
    radius = component.get("radius", 0)
    xlist, ylist = zip(*[(p["x"], p["y"]) for p in [p1, p2, p3]])
    max_bounds = [max(xlist) + radius, max(ylist) + radius]
    min_bounds = [min(xlist) - radius, min(ylist) - radius]

    return min_bounds, max_bounds

## Calculates bounding box for a given **model** with field *components*.
def get_bounding_box_for_model(model):
    min_bound = [None, None]
    max_bound = [None, None]
    if model.get("footprint_type") == "radius":
        r = model.get("radius", 0)
        min_bound = [-r, -r]
        max_bound = [r, r]
    elif model.get("footprint_type") == "points":
        for component in model.get("components"):
            min_bound_new, max_bound_new = get_bounds_for_component(component)
            min_bound = [min(i) for i in zip(min_bound, min_bound_new)]
            max_bound = [max(i) for i in zip(max_bound, max_bound_new)]
    return min_bound, max_bound

def collapse_dictionary(dict_, keylevel=""):
    new_dict = {}
    for key, value in dict_.iteritems():
        if isinstance(value, dict):
            entry = collapse_dictionary(value, keylevel + key + ".")
        else:
            entry = {keylevel + key: value}
        new_dict.update(entry)
    return new_dict
