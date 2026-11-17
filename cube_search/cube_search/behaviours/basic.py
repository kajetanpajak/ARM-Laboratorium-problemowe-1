#!/usr/bin/python3
import py_trees

class InitialiseSearch(py_trees.behaviour.Behaviour):
    """Initialise shared state once for this search.

    Create a fresh behaviour/tree instance for each accepted search goal.
    """

    def __init__(self, expected_count=5, name="Initialise search"):
        super().__init__(name)

        if expected_count < 1:
            raise ValueError("expected_count must be positive")

        self.expected_count = expected_count
        self._done = False

        self.blackboard = self.attach_blackboard_client(
            name=name,
            namespace="search",
        )

        for key in (
            "expected_count",
            "found_cubes",
            "pending_waypoints",
            "current_waypoint",
            "scanned_waypoints",
            "failed_waypoints",
        ):
            self.blackboard.register_key(
                key=key,
                access=py_trees.common.Access.WRITE,
            )

    def update(self):
        # Prevent repeated ticks from clearing an ongoing search.
        if self._done:
            return py_trees.common.Status.SUCCESS

        self.blackboard.expected_count = self.expected_count
        self.blackboard.found_cubes = {}
        self.blackboard.pending_waypoints = []
        self.blackboard.current_waypoint = None
        self.blackboard.scanned_waypoints = []
        self.blackboard.failed_waypoints = []

        self._done = True
        self.feedback_message = (
            f"Search initialised for {self.expected_count} cubes"
        )
        return py_trees.common.Status.SUCCESS
    
    def reset_for_search(self, expected_count):
        self.expected_count = expected_count
        self._done = False


class AllCubesFound(py_trees.behaviour.Behaviour): # question mark
    """Succeed when enough distinct cubes have been confirmed."""

    def __init__(self, name="All cubes found?"):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(
            name=name,
            namespace="search",
        )

        for key in ("expected_count", "found_cubes"):
            self.blackboard.register_key(
                key=key,
                access=py_trees.common.Access.READ,
            )

    def update(self):
        found_count = len(self.blackboard.found_cubes)
        expected_count = self.blackboard.expected_count

        self.feedback_message = (
            f"{found_count}/{expected_count} cubes confirmed"
        )

        if found_count >= expected_count:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE


class SelectNextWaypoint(py_trees.behaviour.Behaviour):
    """Select the first pending waypoint, preserving an active selection."""

    def __init__(self, name="Select next waypoint"):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(
            name=name,
            namespace="search",
        )

        for key in ("pending_waypoints", "current_waypoint"):
            self.blackboard.register_key(
                key=key,
                access=py_trees.common.Access.WRITE,
            )

    def update(self):
        # Re-ticking must not change the goal while the robot is travelling.
        if self.blackboard.current_waypoint is not None:
            self.feedback_message = "Keeping current waypoint"
            return py_trees.common.Status.SUCCESS

        pending = self.blackboard.pending_waypoints

        if not pending:
            self.feedback_message = "No pending waypoints remain"
            return py_trees.common.Status.FAILURE

        # Convert numpy scalar coordinates to ordinary Python floats.
        x, y = pending[0]
        self.blackboard.current_waypoint = (float(x), float(y))
        self.blackboard.pending_waypoints = pending[1:]

        self.feedback_message = (
            f"Selected ({float(x):.2f}, {float(y):.2f})"
        )
        return py_trees.common.Status.SUCCESS


class RecordWaypointOutcome(py_trees.behaviour.Behaviour):
    """Record a completed attempt and clear the current waypoint.

    outcome="scanned": navigation and the observation sweep succeeded.
    outcome="failed": this waypoint is being abandoned.
    """

    def __init__(self, outcome, name=None):
        if outcome not in ("scanned", "failed"):
            raise ValueError("outcome must be 'scanned' or 'failed'")

        super().__init__(name or f"Record waypoint: {outcome}")
        self.outcome = outcome

        self.blackboard = self.attach_blackboard_client(
            name=self.name,
            namespace="search",
        )

        for key in (
            "current_waypoint",
            "scanned_waypoints",
            "failed_waypoints",
        ):
            self.blackboard.register_key(
                key=key,
                access=py_trees.common.Access.WRITE,
            )

    def update(self):
        waypoint = self.blackboard.current_waypoint

        if waypoint is None:
            self.feedback_message = "No current waypoint to record"
            return py_trees.common.Status.FAILURE

        if self.outcome == "scanned":
            self.blackboard.scanned_waypoints = [
                *self.blackboard.scanned_waypoints,
                waypoint,
            ]
        else:
            self.blackboard.failed_waypoints = [
                *self.blackboard.failed_waypoints,
                waypoint,
            ]

        self.blackboard.current_waypoint = None

        self.feedback_message = (
            f"Waypoint {waypoint} recorded as {self.outcome}"
        )
        return py_trees.common.Status.SUCCESS