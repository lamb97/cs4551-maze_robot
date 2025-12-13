#!/usr/bin/env python3
import math
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header


def parse_maze_to_grid(sdf_path, model_name="Maze", res=0.1, margin=0.2):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    ns = ""
    if root.tag.startswith("{"):
        ns = root.tag.split("}")[0] + "}"

    world = root.find(f"{ns}world")
    if world is None:
        raise RuntimeError("No <world> tag found in SDF")
    maze_model = None
    for m in world.findall(f"{ns}model"):
        if m.attrib.get("name") == model_name and m.find(f"{ns}link/{ns}collision") is not None:
            maze_model = m
            break

    if maze_model is None:
        raise RuntimeError(f"Maze model '{model_name}' not found in world")
    maze_pose_elem = maze_model.find(f"{ns}pose")
    if maze_pose_elem is not None:
        mx, my, mz, mroll, mpitch, myaw = map(float, maze_pose_elem.text.split())
    else:
        mx = my = mz = mroll = mpitch = myaw = 0.0

    walls = []
    for link in maze_model.findall(f"{ns}link"):
        pose_elem = link.find(f"{ns}pose")
        coll = link.find(f"{ns}collision")
        if pose_elem is None or coll is None:
            continue

        box = coll.find(f"{ns}geometry/{ns}box")
        if box is None:
            continue

        size_elem = box.find(f"{ns}size")
        if size_elem is None:
            continue

        lx, ly, lz, lroll, lpitch, lyaw = map(float, pose_elem.text.split())
        sx, sy, sz = map(float, size_elem.text.split())

        yaw = myaw + lyaw
        cx = mx + lx
        cy = my + ly

        walls.append((cx, cy, yaw, sx, sy))

    if not walls:
        raise RuntimeError("No wall boxes found in Maze model")
    xs = []
    ys = []
    for cx, cy, yaw, sx, sy in walls:
        half_w = abs(math.cos(yaw)) * sx / 2 + abs(math.sin(yaw)) * sy / 2
        half_h = abs(math.sin(yaw)) * sx / 2 + abs(math.cos(yaw)) * sy / 2
        xs.append(cx - half_w)
        xs.append(cx + half_w)
        ys.append(cy - half_h)
        ys.append(cy + half_h)

    min_x = min(xs) - margin
    max_x = max(xs) + margin
    min_y = min(ys) - margin
    max_y = max(ys) + margin

    width = int(math.ceil((max_x - min_x) / res))
    height = int(math.ceil((max_y - min_y) / res))

    grid = np.zeros((height, width), dtype=np.uint8)  # 0 = free, 1 = wall

    for cx, cy, yaw, sx, sy in walls:
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        half_w = abs(cos_y) * sx / 2 + abs(sin_y) * sy / 2
        half_h = abs(sin_y) * sx / 2 + abs(cos_y) * sy / 2
        x0 = max(min_x, cx - half_w)
        x1 = min(max_x, cx + half_w)
        y0 = max(min_y, cy - half_h)
        y1 = min(max_y, cy + half_h)

        j_min = int((x0 - min_x) / res)
        j_max = int((x1 - min_x) / res)
        i_min = int((y0 - min_y) / res)
        i_max = int((y1 - min_y) / res)

        for i in range(i_min, i_max + 1):
            if i < 0 or i >= height:
                continue
            for j in range(j_min, j_max + 1):
                if j < 0 or j >= width:
                    continue
                x = min_x + (j + 0.5) * res
                y = min_y + (i + 0.5) * res
                dx = x - cx
                dy = y - cy
                local_x = cos_y * dx + sin_y * dy
                local_y = -sin_y * dx + cos_y * dy
                if abs(local_x) <= sx / 2 and abs(local_y) <= sy / 2:
                    grid[i, j] = 1

    origin = (float(min_x), float(min_y))
    return grid, origin, float(res)


class MazeMapPublisher(Node):
    def __init__(self):
        super().__init__("maze_map_publisher")
        self.declare_parameter("sdf_path", "maze.world")
        self.declare_parameter("resolution", 0.1)

        sdf_path = self.get_parameter("sdf_path").get_parameter_value().string_value
        res = self.get_parameter("resolution").get_parameter_value().double_value

        self.get_logger().info(f"Loading maze from: {sdf_path}")
        grid, origin, res = parse_maze_to_grid(sdf_path, res=res)
        self.grid = grid
        self.origin = origin
        self.res = res

        self.height, self.width = self.grid.shape

        self.pub = self.create_publisher(OccupancyGrid, "maze_map", 1)
        self.timer = self.create_timer(1.0, self.publish_map)

    def publish_map(self):
        msg = OccupancyGrid()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        info = MapMetaData()
        info.resolution = self.res
        info.width = self.width
        info.height = self.height
        info.origin.position.x = self.origin[0]
        info.origin.position.y = self.origin[1]
        info.origin.position.z = 0.0
        info.origin.orientation.w = 1.0  # no rotation
        msg.info = info

        # 0 free, 100 occupied, -1 unknown
        data = np.full((self.height, self.width), -1, dtype=np.int8)
        data[self.grid == 0] = 0
        data[self.grid == 1] = 100
        msg.data = data.flatten().tolist()

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MazeMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

