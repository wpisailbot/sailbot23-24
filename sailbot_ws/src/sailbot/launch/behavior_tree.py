
import sailbot.behaviors.sailbot_behavior as tree

##############################################################################
# Launch Service
##############################################################################

def generate_launch_description():
    """
    A ros2 launch script for the Sailbot behavior tree
    """
    return tree.generate_launch_description()