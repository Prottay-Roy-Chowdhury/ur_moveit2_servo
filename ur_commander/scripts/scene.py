import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class AddObstacles(Node):
    def __init__(self):
        super().__init__("add_obstacles")
        self.publisher = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.timer = self.create_timer(1.0, self.add_objects)

    def add_objects(self):
        # Create first collision object (A)
        object_A = CollisionObject()
        object_A.id = "A"
        object_A.header.frame_id = "world"

        A_primitive = SolidPrimitive()
        A_primitive.type = SolidPrimitive.BOX
        A_primitive.dimensions = [0.6, 0.03, 0.6]

        A_pose = Pose()
        A_pose.position.x = 0.0
        A_pose.position.y = 0.20
        A_pose.position.z = 0.0
        A_pose.orientation.w = 1.0

        object_A.primitives.append(A_primitive)
        object_A.primitive_poses.append(A_pose)
        object_A.operation = CollisionObject.ADD

        # Create PlanningScene and add the collision object
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(object_A)
        planning_scene.is_diff = True

        self.publisher.publish(planning_scene)


def main(args=None):
    rclpy.init(args=args)
    node = AddObstacles()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
