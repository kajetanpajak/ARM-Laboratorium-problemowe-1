#!/usr/bin/python3
import numpy as np
import cv2 as cv


class WaypointGenerator:
    def __init__(
        self,
        occupancy_grid: np.ndarray,
        map_resolution,
        map_origin,
        robot_size=0.3,
        peak_neighbourhood=0.75,
        candidate_spacing=0.5,
        target_spacing=0.25,
        scan_range=4.0
    ):
        self.occupancy_grid = occupancy_grid
        
        self.map_resolution = map_resolution
        self.map_origin = map_origin
        
        self.robot_size = robot_size
        self.peak_neighbourhood = peak_neighbourhood
        self.candidate_spacing = candidate_spacing
        self.target_spacing = target_spacing
        self.scan_range = scan_range
        
    def get_waypoints(
        self
    ):  
        free = (self.occupancy_grid == 0) # get rid of the unknown cells
        
        distance = self.distance_transform(free, self.map_resolution)
        
        reachable = self.reachable_mask(distance, self.robot_size)
        
        distance_peaks = self.local_maxima(distance, reachable, self.peak_neighbourhood, self.map_resolution)
        
        grid = self.sample_grid(reachable, self.candidate_spacing, self.map_resolution)
        peaks_set = {point for point in distance_peaks}
        
        grid = np.asarray([p for p in grid if tuple(p) not in peaks_set], dtype=int).reshape(-1, 2)
        
        candidate_waypoints = np.concatenate([distance_peaks, grid])
        
        targets = self.sample_grid(free, self.target_spacing, self.map_resolution)
        
        visible = self.visibility_matrix(free, candidate_waypoints, targets, self.scan_range / self.map_resolution)
        
        selected, uncovered = self.select_waypoints(visible, len(distance_peaks))
        
        selected_waypoints = candidate_waypoints[selected]
        
        map_waypoints = self.grid_to_map(selected_waypoints)

        return map_waypoints
    
    def sample_grid(
        self,
        mask,
        spacing, # in meters
        resolution
    ):
        step = int(round(spacing / resolution))
        
        y, x = np.mgrid[0 : mask.shape[0] : step, 0 : mask.shape[1] : step]
        
        keep = mask[y, x]
        return np.column_stack([x[keep], y[keep]])
    
    def distance_transform(
        self,
        free,
        resolution
    ):
        return cv.distanceTransform(
            np.pad(free.astype(np.uint8), 1),
            cv.DIST_L2,
            cv.DIST_MASK_PRECISE,
        )[1:-1, 1:-1] * resolution
        
    def reachable_mask(
        self,
        distance,
        robot_size, # radius
    ):
        safe = (distance > robot_size).astype(np.uint8)
        
        _, labels, stats, _ = cv.connectedComponentsWithStats(safe, connectivity=4)
        
        largest_component = 1 + np.argmax(stats[1:, cv.CC_STAT_AREA])
        
        return labels == largest_component
    
    def local_maxima(
        self,
        distance,
        reachable,
        peak_neighbourhood,
        resolution
    ):
        radius = int(np.ceil(peak_neighbourhood / resolution)) # radius in pixels
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*radius+1, 2*radius+1))
        
        values = np.where(reachable, distance, 0).astype(np.float32) # keeps only reachable distance transform
        
        peaks = reachable & (values >= cv.dilate(values, kernel) - 1e-6)
        count, labels = cv.connectedComponents(peaks.astype(np.uint8), connectivity=8)
        
        points = []
        
        for label in range(1, count):
            y, x = np.where(labels == label)
            
            max_distance_pixel = distance[y, x].max() # max distance transform value in the component
            max_distance_plateu = distance[y, x] > (max_distance_pixel - 1e-6)
            
            y, x = y[max_distance_plateu], x[max_distance_plateu]
            
            mean = np.argmin((x - x.mean()) ** 2 + (y - y.mean()) ** 2)
            points.append((x[mean], y[mean]))
            
        return points
    
    def visibility_matrix(
        self,
        free, 
        candidates,
        targets,
        range # in pixels
        ):
        
        visible = np.zeros((len(candidates), len(targets)), dtype=bool) # row index is waypoint, column is target grid point
        
        for i, point in enumerate(candidates):
            delta = targets - point
            nearby = np.flatnonzero(np.sum(delta.astype(float) ** 2, axis=1) <= range**2)

            delta = delta[nearby]
            
            steps = int(np.ceil(2 * np.abs(delta).max())) # delta is in pixels!!!
            fractions = np.linspace(0, 1, steps + 1)
            
            rays = point + delta[:, None, :] * fractions[None, :, None]
            pixels = np.floor(rays + 0.5).astype(int)
            
            visible[i, nearby] = free[pixels[:, :, 1], pixels[:, :, 0]].all(axis=1)
            
        return visible
    
    def select_waypoints(
        self,
        visible,
        peak_count
    ):
        uncovered = np.ones(visible.shape[1], dtype=bool) # uncovered target grid points
        selected = []
        preference = np.ones(visible.shape[0])
        preference[:peak_count] = 1.1 # since peaks are first in concatenation
        while uncovered.any() and len(visible):
            gains = visible[:, uncovered].sum(axis=1)
            best = int(np.argmax(gains * preference))
            if gains[best] == 0:
                break
            selected.append(best)
            uncovered &= ~visible[best]
        return np.asarray(selected, dtype=int), uncovered
    
    def grid_to_map(self, waypoints):
        waypoints = np.asarray(waypoints, dtype=float).reshape(-1, 2)
        
        cell_centres = (waypoints + 0.5) * self.map_resolution
        
        return cell_centres + np.array([self.map_origin.position.x, self.map_origin.position.y])