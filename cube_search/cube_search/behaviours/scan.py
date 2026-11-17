import py_trees
import py_trees_ros
from nav2_msgs.action import Spin
import math

class PrepareSpinGoal(py_trees.behaviour.Behaviour):
    """
    """

    def __init__(self, name="Prepare spin goal", frame_id="map"):
        super(PrepareSpinGoal, self).__init__(name)

        self.frame_id = frame_id

        self.blackboard = self.attach_blackboard_client(
            name=name,
            namespace="search",
        )

            
        self.blackboard.register_key(
            "spin_goal",
            py_trees.common.Access.WRITE
        )
        
        self.blackboard.register_key(
            "current_waypoint",
            py_trees.common.Access.READ
        )

    def update(self):
        waypoint = self.blackboard.current_waypoint
        
        if waypoint is None:
            self.feedback_message = "No waypoint selected"
            return py_trees.common.Status.FAILURE
        
        try:
            x, y = waypoint
            x = float(x)
            y = float(y)
            
        except (TypeError, ValueError) as e:
            self.feedback_message = f"Invalid waypoint: {e}"
            return py_trees.common.Status.FAILURE
        
        goal = Spin.Goal()
        goal.target_yaw = 2 * math.pi
        goal.time_allowance.sec = 60
        
        self.blackboard.spin_goal = goal
        
        self.feedback_message = "Prepared a 360 spin"
        return py_trees.common.Status.SUCCESS
    
def create_scan_at_waypoint():
    prepare_goal = PrepareSpinGoal()
    
    spin = py_trees_ros.action_clients.FromBlackboard(
        "Spin at waypoint",
        action_type=Spin,
        action_name="/spin",
        key="/search/spin_goal",
        generate_feedback_message=lambda message: (
            f"Rotated "
            f"{math.degrees(message.feedback.angular_distance_traveled):.1f}"
            f" degrees"
        ),
    )
    
    return py_trees.composites.Sequence(
        name="Navigate to selected waypoint",
        memory=True,
        children=[prepare_goal, spin]
    )