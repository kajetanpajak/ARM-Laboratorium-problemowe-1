import math
from copy import deepcopy
import signal

import py_trees
import py_trees_ros
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from action_msgs.msg import GoalStatus
from rclpy.action import GoalResponse, CancelResponse
from rclpy.task import Future

from action_msgs.msg import GoalStatus
from rclpy.signals import SignalHandlerOptions

from cube_search_interfaces.msg import MarkerDetectionArray, Cube
from cube_search_interfaces.action import SearchCubes

from cube_search.behaviours.basic import (
    InitialiseSearch, AllCubesFound, SelectNextWaypoint, RecordWaypointOutcome
)
from cube_search.behaviours.generate_waypoints import GenerateWaypoints
from cube_search.behaviours.navigate import create_navigate_to_waypoint
from cube_search.behaviours.scan import create_scan_at_waypoint

def create_root(expected_count=5) -> py_trees.behaviour.Behaviour:
    
    waypoint_visit = py_trees.composites.Sequence(
        name="Waypoint visit",
        memory=True,
        children=[
            SelectNextWaypoint(),
            create_navigate_to_waypoint(),
            create_scan_at_waypoint(),
            RecordWaypointOutcome(outcome="scanned")
        ]
    )
    
    repeat_visits = py_trees.decorators.SuccessIsRunning(
        "Keep visiting waypoints",
        child=waypoint_visit
    )
    
    search_work = py_trees.composites.Sequence(
        name="Generate and visit waypoints",
        memory=True,
        children=[
            GenerateWaypoints(),
            repeat_visits
        ]
    )
    
    search_or_finish = py_trees.composites.Selector(
        name="Finish or continue searching",
        memory=False,
        children=[
            AllCubesFound(),
            search_work
        ]
    )
    
    root = py_trees.composites.Sequence(
        name="Search Behaviour Tree",
        memory=True,
        children=[
            InitialiseSearch(expected_count=expected_count),
            search_or_finish,
        ],
    )
    
    return root
    

class SearchServer(Node):
    def __init__(self):
        super(SearchServer, self).__init__("search_server")
        
        self.search_active = False
        self.tree_outcome = None
        
        self.busy = False
        self.active_goal = None
        self.completion_future = None

        self.search_active = False
        self.tree_outcome = None
        self.tree_started = False

        self.stopping = False
        self.finish_reason = None
        self.finish_message = ""
        self.motion_wait = []

        self.search_start_ns = 0
        
        
        ######################
        # ARUCO SUBSCRIPTION #
        ######################
        self.aruco_subscription = self.create_subscription(
            MarkerDetectionArray,
            "/aruco/detections",
            self.aruco_callback,
            10,
        )
        
        ########################
        # SEARCH ACTION SERVER #
        ########################
        self.search_action_server = ActionServer(
            self,
            SearchCubes,
            "search_aruco_cubes",
            execute_callback=self.search_execute_callback,
            goal_callback=self.search_goal_callback,
            cancel_callback=self.search_cancel_callback,
        )
        
        ###################
        # TREE TICK TIMER #
        ###################
        
        root = create_root(expected_count=5)
        self.tree = py_trees_ros.trees.BehaviourTree(root)
        
        self.tree.setup(node=self)
        
        
        self.tree_timer = self.create_timer(
            0.1,
            self.tick_tree,
        )
        
        #####################
        # BLACKBOARD CLIENT #
        #####################
        self.blackboard = py_trees.blackboard.Client(
            name="Search server",
            namespace="search"
        )
        self.blackboard.register_key(
            "found_cubes",
            py_trees.common.Access.WRITE
        )
        
        for key in ("scanned_waypoints", "failed_waypoints"):
            self.blackboard.register_key(
                key=key,
                access=py_trees.common.Access.READ,
            )
        
        
    def search_goal_callback(self, goal_request):
        # Reserve the server immediately, before execute_callback starts.
        if self.busy:
            self.get_logger().warning("Rejecting goal: a search is already active")
            return GoalResponse.REJECT

        if goal_request.expected_count < 1:
            self.get_logger().warning("Rejecting goal: expected_count must be positive")
            return GoalResponse.REJECT

        self.busy = True
        return GoalResponse.ACCEPT


    def search_cancel_callback(self, goal_handle):
        # This approves the request. The timer performs the actual stopping.
        self.get_logger().info("Search cancellation requested")
        return CancelResponse.ACCEPT


    async def search_execute_callback(self, goal_handle):
        self.active_goal = goal_handle

        # This future will receive the SearchCubes.Result from the timer.
        self.completion_future = Future()

        self.tree_outcome = None
        self.tree_started = False
        self.stopping = False
        self.finish_reason = None
        self.finish_message = ""
        self.motion_wait = []

        self.blackboard.found_cubes = {}
        self.search_start_ns = self.get_clock().now().nanoseconds

        try:
            self.tree.root.stop(py_trees.common.Status.INVALID)

            # Reset behaviours which cache data between searches.
            # Add the reset_for_search methods shown below.
            for behaviour in self.tree.root.iterate():
                reset = getattr(behaviour, "reset_for_search", None)
                if callable(reset):
                    reset(expected_count=int(goal_handle.request.expected_count))

            self.search_active = True

        except Exception as error:
            self.get_logger().error(f"Could not initialise search: {error}")
            self.search_active = True
            self.begin_finish("aborted", f"Could not initialise search: {error}")

        # Pause this callback while ROS continues running the timer and
        # subscription callbacks. finish_search() supplies the result.
        return await self.completion_future


    def aruco_callback(self, msg: MarkerDetectionArray):
        # Wait until InitialiseSearch has run, so it cannot erase detections
        # collected before the first tree tick.
        if not self.search_active or not self.tree_started or self.stopping:
            return

        if msg.header.frame_id != "map":
            self.get_logger().warning(
                "Ignoring detections outside the map frame",
                throttle_duration_sec=5.0,
            )
            return

        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )

        # Ignore queued observations from before this search.
        if stamp_ns < self.search_start_ns:
            return

        found = dict(self.blackboard.found_cubes)

        for detection in msg.detections:
            position = detection.pose.position

            if not all(math.isfinite(value) for value in (
                position.x,
                position.y,
                position.z,
            )):
                continue

            marker_id = int(detection.id)

            if marker_id not in found:
                self.get_logger().info(f"Found cube {marker_id}")

            # Repeated observations update the position without increasing
            # the number of distinct cubes.
            found[marker_id] = deepcopy(position)

        self.blackboard.found_cubes = found


    def tick_tree(self):
        if not self.search_active:
            return

        # Cancellation can arrive while searching or while already stopping.
        if self.active_goal.is_cancel_requested:
            if not self.stopping:
                self.begin_finish("canceled", "Search canceled by the client")
            else:
                self.finish_reason = "canceled"
                self.finish_message = "Search canceled by the client"

        if not self.stopping:
            try:
                self.tree.tick()
                self.tree_started = True

                status = self.tree.root.status

                if status == py_trees.common.Status.SUCCESS:
                    self.tree_outcome = status
                    self.begin_finish("succeeded", "Found the requested cubes")

                elif status == py_trees.common.Status.FAILURE:
                    self.tree_outcome = status
                    self.begin_finish(
                        "aborted",
                        "Search tree failed before finding the requested cubes",
                    )

            except Exception as error:
                self.get_logger().error(f"Search tree error: {error}")
                self.begin_finish("aborted", f"Search tree error: {error}")

        feedback = SearchCubes.Feedback()
        feedback.found_count = len(self.blackboard.found_cubes)

        if self.tree_started:
            feedback.visited_waypoints = (
                len(self.blackboard.scanned_waypoints)
                + len(self.blackboard.failed_waypoints)
            )
        else:
            feedback.visited_waypoints = 0

        tip = self.tree.root.tip()
        feedback.current_activity = (
            "Waiting for Nav2 to finish"
            if self.stopping
            else tip.name if tip is not None else "Starting search"
        )

        self.active_goal.publish_feedback(feedback)

        if self.stopping and self.motion_has_finished():
            self.finish_search()
        
    def begin_finish(self, reason, message):
        if self.stopping:
            return

        self.stopping = True
        self.finish_reason = reason
        self.finish_message = message
        self.motion_wait = []

        # Remember outstanding navigation/spin requests before invalidating
        # the tree. A goal may still be awaiting acceptance from Nav2.
        for behaviour in self.tree.root.iterate():
            if not isinstance(
                behaviour,
                py_trees_ros.action_clients.FromBlackboard,
            ):
                continue

            send_future = getattr(behaviour, "send_goal_future", None)

            if send_future is not None:
                self.motion_wait.append({
                    "behaviour": behaviour,
                    "send_future": send_future,
                    "result_future": getattr(
                        behaviour, "get_result_future", None
                    ),
                    "cancel_requested": False,
                })

        # FromBlackboard requests cancellation when interrupted.
        self.tree.root.stop(py_trees.common.Status.INVALID)


    def motion_has_finished(self):
        terminal_statuses = {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }

        all_finished = True

        for item in self.motion_wait:
            try:
                send_future = item["send_future"]

                if not send_future.done():
                    all_finished = False
                    continue

                handle = send_future.result()

                if handle is None:
                    raise RuntimeError("Nav2 returned no goal handle")

                if not handle.accepted:
                    # A rejected goal never started moving the robot.
                    continue

                if item["result_future"] is None:
                    # The behaviour's acceptance callback may have run
                    # since begin_finish().
                    item["result_future"] = getattr(
                        item["behaviour"], "get_result_future", None
                    )

                    if item["result_future"] is None:
                        item["result_future"] = handle.get_result_async()

                result_future = item["result_future"]

                if result_future.done():
                    response = result_future.result()

                    if response.status not in terminal_statuses:
                        raise RuntimeError("Nav2 returned a non-terminal status")

                    continue

                # Also handles cancellation requested before Nav2 accepted
                # the goal. Repeating an earlier cancellation request is OK.
                if not item["cancel_requested"]:
                    item["cancel_future"] = handle.cancel_goal_async()
                    item["cancel_requested"] = True

                all_finished = False

            except Exception as error:
                # Do not report completion when motion status is unknown.
                self.get_logger().error(
                    f"Cannot confirm Nav2 has finished: {error}",
                    throttle_duration_sec=5.0,
                )
                all_finished = False

        return all_finished


    def finish_search(self):
        result = SearchCubes.Result()
        result.header.frame_id = "map"
        result.header.stamp = self.get_clock().now().to_msg()
        result.message = self.finish_message

        for marker_id, position in sorted(self.blackboard.found_cubes.items()):
            cube = Cube()
            cube.id = int(marker_id)
            cube.position = deepcopy(position)
            result.cubes.append(cube)

        if self.finish_reason == "succeeded":
            self.active_goal.succeed()
        elif self.finish_reason == "canceled":
            self.active_goal.canceled()
        else:
            self.active_goal.abort()

        self.search_active = False
        self.stopping = False

        # This wakes the execute callback, which returns this result.
        self.completion_future.set_result(result)

        self.active_goal = None
        self.busy = False
        
        
def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )

    executor = SingleThreadedExecutor()
    node = None
    stop_requested = False

    def request_shutdown(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    previous_sigint = signal.signal(signal.SIGINT, request_shutdown)
    previous_sigterm = signal.signal(signal.SIGTERM, request_shutdown)

    try:
        node = SearchServer()
        node.shutting_down = False
        executor.add_node(node)

        node.get_logger().info("Search server ready")

        while rclpy.ok():
            if stop_requested:
                if not node.shutting_down:
                    node.shutting_down = True
                    node.get_logger().info(
                        "Shutting down: waiting for any active search to stop"
                    )

                if node.search_active and not node.stopping:
                    node.begin_finish(
                        "aborted",
                        "Search interrupted by server shutdown",
                    )

                # Also waits for an accepted goal whose execute callback
                # has not started yet.
                if not node.busy:
                    break

            # Processes timer, detection, and action callbacks.
            executor.spin_once(timeout_sec=0.1)

    finally:
        executor.shutdown()

        try:
            if node is not None:
                node.tree_timer.cancel()
                node.search_action_server.destroy()

                # Shuts down behaviours and destroys the shared ROS node.
                node.tree.shutdown()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

            signal.signal(signal.SIGINT, previous_sigint)
            signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    main()