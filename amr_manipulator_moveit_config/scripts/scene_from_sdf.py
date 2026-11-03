#!/usr/bin/env python3
"""Load the boxes of the warehouse SDF into MoveIt's planning scene (frame: map)."""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

FRAME = 'map'    # the world frame of the SDF: the map frame, since the robot starts at the origin


def pose(text):
    """'x y z r p y' -> (x, y, z, yaw); the world has no tilted box, so only the yaw is kept."""
    x, y, z, r, p, yaw = [float(v) for v in (text or '0 0 0 0 0 0').split()]
    if abs(r) > 1e-9 or abs(p) > 1e-9:
        raise ValueError('a box rotated about x or y is not supported')
    return x, y, z, yaw


def compose(parent, child):
    """Pose of `child` (given in `parent`) in the frame of `parent`'s parent, planar rotation only."""
    px, py, pz, pyaw = parent
    cx, cy, cz, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return px + c * cx - s * cy, py + s * cx + c * cy, pz + cz, pyaw + cyaw


def collision_objects(sdf_path):
    """One CollisionObject per model of the world, with a box for every box collision. Planes are skipped."""
    objects = []
    for model in ET.parse(sdf_path).getroot().find('world').findall('model'):
        obj = CollisionObject()
        obj.id = model.get('name')
        obj.header.frame_id = FRAME
        obj.operation = CollisionObject.ADD
        for link in model.findall('link'):
            link_pose = compose(pose(model.findtext('pose')), pose(link.findtext('pose')))
            for col in link.findall('collision'):
                box = col.find('geometry/box')
                if box is None:
                    continue
                x, y, z, yaw = compose(link_pose, pose(col.findtext('pose')))
                prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in box.findtext('size').split()])
                at = Pose()
                at.position.x, at.position.y, at.position.z = x, y, z
                at.orientation.z, at.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
                obj.primitives.append(prim)
                obj.primitive_poses.append(at)
        if obj.primitives:
            objects.append(obj)
    return objects


def main():
    default_sdf = os.path.join(get_package_share_directory('amr_manipulator_gazebo'), 'worlds', 'warehouse.sdf')
    ap = argparse.ArgumentParser()
    ap.add_argument('--sdf', default=default_sdf)
    ap.add_argument('--timeout', type=float, default=60.0, help='seconds to wait for move_group')
    args, _ = ap.parse_known_args()

    objects = collision_objects(args.sdf)
    rclpy.init()
    node = Node('scene_from_sdf')
    client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    if not client.wait_for_service(timeout_sec=args.timeout):
        node.get_logger().error('/apply_planning_scene is not available: is move_group running?')
        sys.exit(1)
    request = ApplyPlanningScene.Request()
    request.scene = PlanningScene(is_diff=True)
    request.scene.world.collision_objects = objects
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    ok = future.result() is not None and future.result().success
    boxes = sum(len(o.primitives) for o in objects)
    node.get_logger().info(f'{len(objects)} objects, {boxes} boxes from {os.path.basename(args.sdf)} '
                           f'{"applied to the planning scene" if ok else "NOT applied"}')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
