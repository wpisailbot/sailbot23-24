import py_trees
import py_trees_ros
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import launch
import launch_ros
import sys

def generate_launch_description():
    """
    Launcher for the Sailbot behavior tree.

    Returns:
        the launch description
    """
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package='sailbot',
                executable="main_behavior",
                output='screen',
                emulate_tty=True,
            )
        ]
    )

def sailbot_create_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Parallel(
        name="Sailbot",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )
    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        threshold=30.0
    )

    priorities = py_trees.composites.Selector(name="Tasks", memory=False)
    idle = py_trees.behaviours.Running(name="Idle")

    root.add_child(topics2bb)
    topics2bb.add_child(battery2bb)
    root.add_child(priorities)
    priorities.add_child(idle)

    return root


def main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = sailbot_create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="foo", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()