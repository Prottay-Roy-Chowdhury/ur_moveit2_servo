import rclpy
import time
from rclpy.node import Node
from rclpy.logging import get_logger
from pathlib import Path

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder


class URCommander:
    def __init__(self):
        # ... your existing init code ...
        moveit_config = (
            MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
            .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur10e"})
            .moveit_cpp(Path("config") / "moveit_cpp.yaml")
            .to_moveit_configs()
        ).to_dict()
        self.ur = MoveItPy(node_name="ur", config_dict=moveit_config)
        self.arm = self.ur.get_planning_component("ur_manipulator")
        self.scene_monitor = self.ur.get_planning_scene_monitor()
        with self.scene_monitor.read_only() as scene:
            robot_state = scene.current_state
            print("Robot state:", robot_state)
        self.logger = get_logger("ur_commander")
        self.logger.info("URCommander initialized")

    def add_collision_objects(self):
        object_positions = [
            (0.15, 0.1, 0.5),
            (0.25, 0.0, 1.0),
            (-0.25, -0.3, 0.8),
            (0.25, 0.3, 0.75),
        ]
        object_dimensions = [
            (0.1, 0.5, 0.1),
            (0.1, 0.4, 0.1),
            (0.2, 0.2, 0.2),
            (0.15, 0.15, 0.15),
        ]

        with self.scene_monitor.read_write() as scene:
            collision_object = CollisionObject()
            collision_object.header.frame_id = "base_link"
            collision_object.id = "boxes"

            for pos, dims in zip(object_positions, object_dimensions):
                box_pose = Pose()
                box_pose.position.x, box_pose.position.y, box_pose.position.z = pos

                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = dims

                collision_object.primitives.append(box)
                collision_object.primitive_poses.append(box_pose)
                collision_object.operation = CollisionObject.ADD

            scene.apply_collision_object(collision_object)
            scene.current_state.update()

        self.logger.info("Collision objects added to the scene")

    def shutdown(self):
        self.ur.shutdown()
        rclpy.shutdown()
        self.logger.info("Shutdown complete")


if __name__ == "__main__":
    rclpy.init()
    commander = URCommander()
    commander.add_collision_objects()
    try:
        while True:
            time.sleep(1)  # Keep alive
    except KeyboardInterrupt:
        pass
    commander.shutdown()
