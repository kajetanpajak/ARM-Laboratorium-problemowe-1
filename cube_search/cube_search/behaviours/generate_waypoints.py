#!/usr/bin/python3
import py_trees
import py_trees_ros
import numpy as np
from cube_search.behaviours.waypoint_generator import WaypointGenerator
from nav_msgs.srv import GetMap
from concurrent.futures import ThreadPoolExecutor

class GenerateWaypoints(py_trees.behaviour.Behaviour):
    """
    """

    def __init__(
        self,
        name="Generate waypoints",
        map_service_name: str="/map_server/map"
        ):
        super(GenerateWaypoints, self).__init__(name=name)

        self._done = False
        self.map_future = None
        self.map = None
        
        self.generation_future = None

        self.blackboard = self.attach_blackboard_client(
            name=name,
            namespace="search",
        )
        
        self.map_service_name = map_service_name

        for key in (
            "pending_waypoints",
        ):
            self.blackboard.register_key(
                key=key,
                access=py_trees.common.Access.WRITE,
            )

    def setup(self, **kwargs):
        self.logger.debug("{}.setup()".format(self.qualified_name))
        
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                f"{self.qualified_name}: setup() requires a 'node' argument"
            ) from e
        
        self.map_service_client = self.node.create_client(
            GetMap,
            self.map_service_name
        )
        
        self.worker = ThreadPoolExecutor(max_workers=1)
        
        self.feedback_message = "Created map service client"
        
    def update(self):
        if self._done:
            return py_trees.common.Status.SUCCESS
        
        if self.map_future is None:
            if not self.map_service_client.service_is_ready():
                self.feedback_message = "Waiting for map service"
                return py_trees.common.Status.RUNNING
            
            self.map_future = self.map_service_client.call_async(GetMap.Request())
            self.feedback_message = "Requesting map"
            return py_trees.common.Status.RUNNING
        
        if not self.map_future.done():
            self.feedback_message = "Waiting for map response"
            return py_trees.common.Status.RUNNING
        
        try:
            response = self.map_future.result()
            if response is None:
                raise RuntimeError("Map service returned no response")
            
        except Exception as error:
            self.feedback_message = f"Map request failed: {error}"
            return py_trees.common.Status.FAILURE
        
        if self.generation_future is None:
            self.map_msg = response.map
            
            waypoint_generator = WaypointGenerator(
                occupancy_grid=np.asarray(self.map_msg.data, dtype=np.float32).reshape(
                    self.map_msg.info.height, self.map_msg.info.width
                ),
                map_resolution=self.map_msg.info.resolution,
                map_origin=self.map_msg.info.origin
            )
            
            self.generation_future = self.worker.submit(
                waypoint_generator.get_waypoints
            )

            self.feedback_message = "Generating waypoints"
            return py_trees.common.Status.RUNNING
        
        if not self.generation_future.done():
            self.feedback_message = "Generating waypoints"
            return py_trees.common.Status.RUNNING
        
        try:
            generated_waypoints = self.generation_future.result()
            
            waypoints = [
                (float(x), float(y)) for x, y in generated_waypoints
            ]
            
            if not waypoints:
                raise RuntimeError("No waypoints generated")
            
            
        except Exception as error:
            self.feedback_message = f"Waypoint generation failed: {error}"
            return py_trees.common.Status.FAILURE
        
        self.blackboard.pending_waypoints = waypoints
        self._done = True
        
        self.feedback_message = f"Generated {len(waypoints)} waypoints"
        
        return py_trees.common.Status.SUCCESS
    
    def reset_for_search(self, expected_count):
        if self.map_future is not None and not self.map_future.done():
            self.map_future.cancel()

        if (
            self.generation_future is not None
            and not self.generation_future.done()
        ):
            self.generation_future.cancel()

        self.map_future = None
        self.generation_future = None
        self.map_msg = None
        self._done = False