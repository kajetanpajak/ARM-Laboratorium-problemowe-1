import py_trees
import py_trees_ros
from nav2_msgs.action import NavigateToPose


class PrepareNavigationGoal(py_trees.behaviour.Behaviour):
    """
    """

    def __init__(self, name="Prepare navigation goal", frame_id="map"):
        super(PrepareNavigationGoal, self).__init__(name)

        self.frame_id = frame_id

        self.blackboard = self.attach_blackboard_client(
            name=name,
            namespace="search",
        )

        self.blackboard.register_key(
            "current_waypoint",
            access=py_trees.common.Access.READ
        )
        
        self.blackboard.register_key(
            "navigation_goal",
            py_trees.common.Access.WRITE
        )
        
    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                f"{self.qualified_name}: setup() requires a 'node' argument"
            ) from e

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
        
        goal = NavigateToPose.Goal()
        
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        
        goal.pose.pose.orientation.w = 1.0
        
        self.blackboard.navigation_goal = goal
        
        self.feedback_message = f"Prepared NavigateToPose goal: ({x:.2f}, {y:.2f})"
        return py_trees.common.Status.SUCCESS
    
def create_navigate_to_waypoint():
    prepare_goal = PrepareNavigationGoal()
    
    navigate = py_trees_ros.action_clients.FromBlackboard(
        "Navigate to waypoint",
        action_type=NavigateToPose,
        action_name="/navigate_to_pose",
        key="/search/navigation_goal",
        generate_feedback_message=lambda message: (f"{message.feedback.distance_remaining:.2f} m remaining")
    )
    
    return py_trees.composites.Sequence(
        name="Navigate to selected waypoint",
        memory=True,
        children=[prepare_goal, navigate]
    )