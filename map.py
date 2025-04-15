import numpy as np
import matplotlib.pyplot as plt
import random
import math

class Map:
    def __init__(self, size, obstacle_density=0.2, elevation_variance=0.3):
        self.size = size
        # Grid representation: 0 = road (traversable), 1 = obstacle, 2 = empty space (non-traversable)
        self.grid = np.zeros((size, size), dtype=int)
        self.elevation = np.zeros((size, size), dtype=float)
        self.terrain_type = np.zeros((size, size), dtype=int)  # 0: flat, 1: uphill, 2: downhill
        self.generate_map(obstacle_density, elevation_variance)
        
    def generate_map(self, obstacle_density, elevation_variance):
        # Generate obstacles
        for i in range(self.size):
            for j in range(self.size):
                if random.random() < obstacle_density:
                    self.grid[i][j] = 1
        
        # Generate elevation (using simplex noise or similar would be better)
        # For simplicity, we'll use a simple approach here
        for i in range(self.size):
            for j in range(self.size):
                if self.grid[i][j] == 0:  # Only set elevation for road cells
                    # Simple elevation pattern - can be replaced with complex terrain
                    self.elevation[i][j] = random.uniform(-elevation_variance, elevation_variance)
        
        # Calculate terrain types
        self.calculate_terrain_types()
    
    def generate_city_map(self, road_density=0.3, block_size=5, elevation_variance=0.3):
        """
        Generate a city-like map with variable grid spacing and 1-unit wide roads.
        Ensures start and end points are far apart with interesting path choices.
        """
        # Reset grid and elevation - initialize all to empty space (2)
        self.grid = np.full((self.size, self.size), 2, dtype=int)  # 2 = empty space
        self.elevation = np.zeros((self.size, self.size), dtype=float)
        self.terrain_type = np.zeros((self.size, self.size), dtype=int)
        
        # Determine grid line positions with variable spacing
        # For horizontal roads
        h_roads = []
        current_pos = 0
        
        # Generate positions dynamically based on map size with variability
        min_spacing = max(3, self.size // 25)  # Minimum spacing between roads
        max_spacing = max(8, self.size // 12)  # Maximum spacing between roads
        
        while current_pos < self.size:
            h_roads.append(current_pos)
            spacing = random.randint(min_spacing, max_spacing)
            current_pos += spacing
        
        # Make sure the last road is at the bottom edge
        if h_roads[-1] != self.size - 1:
            h_roads.append(self.size - 1)
        
        # For vertical roads with similar variable spacing
        v_roads = []
        current_pos = 0
        
        while current_pos < self.size:
            v_roads.append(current_pos)
            spacing = random.randint(min_spacing, max_spacing)
            current_pos += spacing
        
        # Make sure the last road is at the right edge
        if v_roads[-1] != self.size - 1:
            v_roads.append(self.size - 1)
        
        # Create the grid roads (mark as 0)
        for h in h_roads:
            self.grid[h, :] = 0  # Horizontal road
        
        for v in v_roads:
            self.grid[:, v] = 0  # Vertical road
        
        # Add some diagonal/curved roads for more interesting pathfinding
        num_diagonals = max(2, self.size // 20)
        for _ in range(num_diagonals):
            # Pick two random road intersections
            h1 = random.choice(h_roads)
            v1 = random.choice(v_roads)
            h2 = random.choice(h_roads)
            v2 = random.choice(v_roads)
            
            # Ensure they're not the same point
            if (h1, v1) != (h2, v2):
                # Create a path between these points
                # Bresenham's line algorithm for drawing a line
                x1, y1 = v1, h1
                x2, y2 = v2, h2
                
                dx = abs(x2 - x1)
                dy = abs(y2 - y1)
                sx = 1 if x1 < x2 else -1
                sy = 1 if y1 < y2 else -1
                err = dx - dy
                
                while True:
                    # Set this point as a road
                    self.grid[y1, x1] = 0
                    
                    if x1 == x2 and y1 == y2:
                        break
                    
                    e2 = 2 * err
                    if e2 > -dy:
                        err -= dy
                        x1 += sx
                    if e2 < dx:
                        err += dx
                        y1 += sy
        
        # Calculate blocks (areas between roads)
        blocks = []
        for i in range(len(h_roads) - 1):
            for j in range(len(v_roads) - 1):
                top = h_roads[i] + 1
                bottom = h_roads[i + 1] - 1
                left = v_roads[j] + 1
                right = v_roads[j + 1] - 1
                
                if top <= bottom and left <= right:
                    blocks.append((top, left, bottom, right))
        
        # Fill blocks with buildings (1) or leave as empty space (2)
        for block in blocks:
            top, left, bottom, right = block
            # 70% chance of buildings, 30% empty space
            fill_type = 1 if random.random() < 0.7 else 2
            
            for i in range(top, bottom + 1):
                for j in range(left, right + 1):
                    self.grid[i, j] = fill_type
        
        # Determine start and end points that are far apart
        # Choose from corners or edges to ensure maximum distance
        corner_pairs = [
            ((0, 0), (self.size - 1, self.size - 1)),  # Top-left to bottom-right
            ((0, self.size - 1), (self.size - 1, 0)),  # Top-right to bottom-left
            ((0, self.size // 2), (self.size - 1, self.size // 2)),  # Top-middle to bottom-middle
            ((self.size // 2, 0), (self.size // 2, self.size - 1)),  # Middle-left to middle-right
            ((self.size // 4, self.size // 4), (self.size * 3 // 4, self.size * 3 // 4))  # Interior diagonal
        ]
        
        # Set start and end points, ensuring they're on roads
        start, end = random.choice(corner_pairs)
        
        # Find nearest road points if not already on a road
        start = self.find_nearest_road_point(start)
        end = self.find_nearest_road_point(end)
        
        self.start_point = start
        self.end_point = end
        
        # Ensure start and end are on roads
        self.grid[start[0], start[1]] = 0
        self.grid[end[0], end[1]] = 0
        
        # Generate elevation with interesting patterns
        self.generate_interesting_elevation(elevation_variance)
        
        # Add strategic hills and valleys along roads
        self.add_strategic_elevation_features(elevation_variance)
        
        # Ensure a path exists from start to goal
        self.ensure_path_exists(start, end)
        
        # Calculate terrain types based on elevation gradients (only for roads)
        self.calculate_terrain_types()
    
    def find_nearest_road_point(self, point):
        """Find the nearest road point to the given point"""
        x, y = point
        
        # Check if already on a road
        if self.grid[x, y] == 0:
            return point
        
        # Search in expanding circles until a road is found
        for radius in range(1, self.size):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    # Only check points at approximately radius distance
                    if abs(dx) + abs(dy) == radius:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < self.size and 0 <= ny < self.size and self.grid[nx, ny] == 0:
                            return (nx, ny)
        
        # Fallback to original point (shouldn't happen if there are roads)
        return point
    
    def generate_interesting_elevation(self, elevation_variance):
        """Generate a more interesting elevation pattern with multiple features"""
        # Create perlin-like noise for base elevation
        noise_grid = np.zeros((self.size, self.size))
        
        # Generate several random "blobs" of elevation
        num_features = max(5, self.size // 15)
        for _ in range(num_features):
            center_x = random.randint(0, self.size - 1)
            center_y = random.randint(0, self.size - 1)
            radius = random.randint(self.size // 10, self.size // 4)
            height = random.random() * 2 - 1  # -1 to 1
            
            # Apply to noise grid
            for x in range(max(0, center_x - radius), min(self.size, center_x + radius + 1)):
                for y in range(max(0, center_y - radius), min(self.size, center_y + radius + 1)):
                    distance = math.sqrt((x - center_x)**2 + (y - center_y)**2)
                    if distance < radius:
                        # Smooth falloff based on distance
                        factor = (1 - distance / radius) ** 2
                        noise_grid[x, y] += height * factor
        
        # Normalize noise grid to 0-1 range
        if np.max(noise_grid) > np.min(noise_grid):
            noise_grid = (noise_grid - np.min(noise_grid)) / (np.max(noise_grid) - np.min(noise_grid))
        
        # Apply elevation only to roads
        road_mask = (self.grid == 0)
        self.elevation = np.zeros((self.size, self.size))
        self.elevation[road_mask] = noise_grid[road_mask] * elevation_variance + (1 - elevation_variance/2)
        
        # Add a slight penalty to edge roads to discourage edge-hugging
        edge_penalty = 0.2
        # Top and bottom edges
        for x in range(self.size):
            if self.grid[0, x] == 0:  # Top edge
                self.elevation[0, x] += edge_penalty
            if self.grid[self.size-1, x] == 0:  # Bottom edge
                self.elevation[self.size-1, x] += edge_penalty
                
        # Left and right edges
        for y in range(self.size):
            if self.grid[y, 0] == 0:  # Left edge
                self.elevation[y, 0] += edge_penalty
            if self.grid[y, self.size-1] == 0:  # Right edge
                self.elevation[y, self.size-1] += edge_penalty
    
    def add_strategic_elevation_features(self, elevation_variance):
        """Add strategic hills and valleys along roads"""
        # Number of features based on map size
        num_hills = max(2, self.size // 15)
        num_valleys = max(2, self.size // 15)
        
        # Create a list of road intersections (good places for features)
        road_intersections = []
        for i in range(self.size):
            for j in range(self.size):
                if self.grid[i, j] == 0:
                    # Check if it's an intersection (has multiple road neighbors)
                    road_neighbors = 0
                    for di, dj in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                        ni, nj = i + di, j + dj
                        if 0 <= ni < self.size and 0 <= nj < self.size and self.grid[ni, nj] == 0:
                            road_neighbors += 1
                    
                    if road_neighbors > 2:  # Intersection
                        road_intersections.append((i, j))
        
        # If no intersections found, use regular road cells
        if not road_intersections:
            road_intersections = [(i, j) for i in range(self.size) for j in range(self.size) 
                                if self.grid[i, j] == 0]
        
        # Add hills at strategic road intersections
        for _ in range(num_hills):
            if not road_intersections:
                break
                
            # Choose a random intersection
            intersection_idx = random.randrange(len(road_intersections))
            center_x, center_y = road_intersections.pop(intersection_idx)
            
            # Ensure it's not too close to start or end
            start_dist = math.sqrt((center_x - self.start_point[0])**2 + 
                                  (center_y - self.start_point[1])**2)
            end_dist = math.sqrt((center_x - self.end_point[0])**2 + 
                                (center_y - self.end_point[1])**2)
            
            min_dist = min(self.size / 6, 10)  # Minimum distance from start/end
            if start_dist < min_dist or end_dist < min_dist:
                continue  # Too close to start/end
            
            radius = random.randint(self.size // 20, self.size // 10)
            height = random.uniform(0.3, 0.7)  # Higher hills for more impact
            
            self.add_hill(center_x, center_y, radius, height)
        
        # Reset intersection list for valleys
        road_intersections = [(i, j) for i in range(self.size) for j in range(self.size) 
                            if self.grid[i, j] == 0 and self.elevation[i, j] > 0.4]
        
        # Add valleys at strategic locations
        for _ in range(num_valleys):
            if not road_intersections:
                break
                
            # Choose a random road position (preferably with higher elevation)
            intersection_idx = random.randrange(len(road_intersections))
            center_x, center_y = road_intersections.pop(intersection_idx)
            
            # Check distance from start/end
            start_dist = math.sqrt((center_x - self.start_point[0])**2 + 
                                  (center_y - self.start_point[1])**2)
            end_dist = math.sqrt((center_x - self.end_point[0])**2 + 
                                (center_y - self.end_point[1])**2)
            
            min_dist = min(self.size / 6, 10)
            if start_dist < min_dist or end_dist < min_dist:
                continue
            
            radius = random.randint(self.size // 20, self.size // 10)
            depth = random.uniform(0.2, 0.5)
            
            self.add_valley(center_x, center_y, radius, depth)
    
    def add_hill(self, center_x, center_y, radius, height):
        """Add a hill centered at (center_x, center_y) affecting only roads"""
        x_start = max(0, center_x - radius)
        x_end = min(self.size, center_x + radius + 1)
        y_start = max(0, center_y - radius)
        y_end = min(self.size, center_y + radius + 1)
        
        for x in range(x_start, x_end):
            for y in range(y_start, y_end):
                # Only modify elevation for road cells
                if self.grid[x, y] == 0:
                    # Calculate distance from center
                    distance = math.sqrt((x - center_x)**2 + (y - center_y)**2)
                    
                    if distance < radius:
                        # Cosine-based smooth falloff
                        elevation_change = height * math.cos(math.pi * distance / (2 * radius))
                        self.elevation[x, y] += max(0, elevation_change)
    
    def add_valley(self, center_x, center_y, radius, depth):
        """Add a valley centered at (center_x, center_y) affecting only roads"""
        x_start = max(0, center_x - radius)
        x_end = min(self.size, center_x + radius + 1)
        y_start = max(0, center_y - radius)
        y_end = min(self.size, center_y + radius + 1)
        
        for x in range(x_start, x_end):
            for y in range(y_start, y_end):
                # Only modify elevation for road cells
                if self.grid[x, y] == 0:
                    # Calculate distance from center
                    distance = math.sqrt((x - center_x)**2 + (y - center_y)**2)
                    
                    if distance < radius:
                        # Cosine-based smooth falloff
                        elevation_change = depth * math.cos(math.pi * distance / (2 * radius))
                        self.elevation[x, y] -= max(0, elevation_change)
                        # Ensure elevation doesn't go negative
                        self.elevation[x, y] = max(0, self.elevation[x, y])
    
    def ensure_path_exists(self, start, goal):
        """Ensure that a path exists from start to goal, modifying grid and elevation as needed"""
        x1, y1 = start
        x2, y2 = goal
        
        # If both points are in the same row or column, they're already connected
        if x1 == x2 or y1 == y2:
            return
        
        # First check if a path already exists using simple BFS
        if self.check_path_exists(start, goal):
            return
            
        # If there isn't a path already, we need to make sure one exists
        # Find the nearest road points in grid rows or columns
        road_points_x1 = []
        for y in range(self.size):
            if self.grid[x1, y] == 0 and y != y1:  # Avoid the start point itself
                road_points_x1.append((x1, y))
                
        road_points_y2 = []
        for x in range(self.size):
            if self.grid[x, y2] == 0 and x != x2:  # Avoid the goal point itself
                road_points_y2.append((x, y2))
        
        # Try to connect points if found
        if road_points_x1 and road_points_y2:
            # Find closest pairs
            closest_dist = float('inf')
            closest_pair = None
            
            for p1 in road_points_x1:
                for p2 in road_points_y2:
                    dist = abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
                    if dist < closest_dist:
                        closest_dist = dist
                        closest_pair = (p1, p2)
            
            if closest_pair:
                # Create a road connecting these points
                p1, p2 = closest_pair
                
                # Use Bresenham's line algorithm
                x1, y1 = p1
                x2, y2 = p2
                
                dx = abs(x2 - x1)
                dy = abs(y2 - y1)
                sx = 1 if x1 < x2 else -1
                sy = 1 if y1 < y2 else -1
                err = dx - dy
                
                while True:
                    # Set this point as a road
                    self.grid[x1, y1] = 0
                    
                    if x1 == x2 and y1 == y2:
                        break
                    
                    e2 = 2 * err
                    if e2 > -dy:
                        err -= dy
                        x1 += sx
                    if e2 < dx:
                        err += dx
                        y1 += sy
        
        # As a fallback, create a direct path if still needed
        if not self.check_path_exists(start, goal):
            # Create a direct path between start and goal
            x1, y1 = start
            x2, y2 = goal
            
            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            sx = 1 if x1 < x2 else -1
            sy = 1 if y1 < y2 else -1
            err = dx - dy
            
            while True:
                # Set this point as a road
                self.grid[x1, y1] = 0
                
                if x1 == x2 and y1 == y2:
                    break
                
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    x1 += sx
                if e2 < dx:
                    err += dx
                    y1 += sy
        
        # Smooth elevation along the path to make it traversable
        self.smooth_path_elevation(start, goal)
    
    def smooth_path_elevation(self, start, goal):
        """Smooth elevation changes along the path from start to goal using A* to find the path"""
        # Find the path using a simple A* without battery constraints
        path = self.find_path(start, goal)
        
        if not path:
            return  # No path found
            
        # Set a more moderate elevation along the path
        for i in range(len(path)):
            x, y = path[i]
            # Set to a moderate value (neither too high nor too low)
            self.elevation[x, y] = 0.5
            
        # Then add slight gradient for more interesting energy decisions
        path_length = len(path)
        if path_length > 2:
            for i in range(1, path_length - 1):
                x, y = path[i]
                # Create slight undulations along the path
                position_factor = i / path_length
                # Sine wave pattern for gentle up/down
                self.elevation[x, y] += 0.1 * math.sin(position_factor * math.pi * 4)
    
    def check_path_exists(self, start, goal):
        """Check if a path exists from start to goal using BFS"""
        visited = set()
        queue = [start]
        visited.add(start)
        
        while queue:
            current = queue.pop(0)
            
            if current == goal:
                return True
                
            # Check neighbors
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)
                
                if (0 <= nx < self.size and 0 <= ny < self.size and 
                    self.grid[nx, ny] == 0 and neighbor not in visited):
                    queue.append(neighbor)
                    visited.add(neighbor)
        
        return False
    
    def find_path(self, start, goal):
        """Find a path from start to goal using A* algorithm"""
        # Simple A* implementation for finding a path
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        open_set_hash = {start}
        
        while open_set:
            _, current = min(open_set, key=lambda x: x[0])
            open_set.remove((f_score[current], current))
            open_set_hash.remove(current)
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                nx, ny = neighbor
                
                if not (0 <= nx < self.size and 0 <= ny < self.size and self.grid[nx, ny] == 0):
                    continue
                
                tentative_g_score = g_score[current] + math.sqrt(dx**2 + dy**2)
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    if neighbor not in open_set_hash:
                        open_set.append((f_score[neighbor], neighbor))
                        open_set_hash.add(neighbor)
        
        return None  # No path found
    
    def heuristic(self, a, b):
        """Calculate heuristic distance between two points"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def has_adjacent_road(self, x, y):
        """Check if a position has at least one adjacent road cell"""
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue  # Skip the cell itself
                
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.size and 0 <= ny < self.size and self.grid[nx, ny] == 0:
                    return True
        
        return False
    
    def is_valid_position(self, x, y):
        """Check if position is a valid road cell"""
        return 0 <= x < self.size and 0 <= y < self.size and self.grid[x, y] == 0
    
    def get_neighbors(self, x, y):
        """Get valid road neighbors of a position"""
        # 8-directional movement
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # 4-directional
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # diagonals
        ]
        
        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if self.is_valid_position(nx, ny):
                neighbors.append((nx, ny))
        
        return neighbors
    
    def calculate_terrain_types(self):
        """Calculate terrain types (flat, uphill, downhill) based on elevation gradients"""
        # Initialize terrain_type array
        self.terrain_type = np.zeros((self.size, self.size), dtype=int)  # 0: flat, 1: uphill, 2: downhill
        
        # Calculate elevation gradients - only for road cells
        gradient_threshold = 0.05  # Threshold for considering a slope significant
        
        for i in range(self.size):
            for j in range(self.size):
                if self.grid[i, j] != 0:  # Skip non-road cells
                    continue
                    
                # Calculate average gradient with neighbors
                gradients = []
                for di, dj in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                    ni, nj = i + di, j + dj
                    if 0 <= ni < self.size and 0 <= nj < self.size and self.grid[ni, nj] == 0:
                        gradients.append(self.elevation[ni, nj] - self.elevation[i, j])
                
                if not gradients:
                    continue
                    
                avg_gradient = sum(gradients) / len(gradients)
                
                if abs(avg_gradient) < gradient_threshold:
                    self.terrain_type[i, j] = 0  # Flat
                elif avg_gradient > 0:
                    self.terrain_type[i, j] = 1  # Uphill
                else:
                    self.terrain_type[i, j] = 2  # Downhill
    
    def calculate_slope(self, from_pos, to_pos):
        """Calculate slope in degrees between two positions"""
        x1, y1 = from_pos
        x2, y2 = to_pos
        
        # Get elevations
        elev1 = self.elevation[x1, y1]
        elev2 = self.elevation[x2, y2]
        
        # Calculate elevation difference
        elevation_diff = elev2 - elev1
        
        # Calculate horizontal distance
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        # Calculate slope angle (in degrees)
        if distance > 0:
            slope_degrees = math.degrees(math.atan2(elevation_diff, distance))
        else:
            slope_degrees = 0
            
        return slope_degrees
    
    def visualize(self, start=None, goal=None, standard_path=None, battery_path=None, filename=None):
        """Create a detailed visualization of the map with paths and elevation information"""
        plt.figure(figsize=(12, 10))
        
        # Create a custom colormap for different cell types
        # Create masked arrays for each type
        road_mask = (self.grid == 0)
        obstacle_mask = (self.grid == 1)
        
        # Create elevation map for roads only
        road_elevation = np.ma.masked_where(~road_mask, self.elevation)
        
        # Plot road elevation with terrain colormap
        im = plt.imshow(road_elevation, cmap='terrain', interpolation='nearest')
        
        # Plot buildings/obstacles
        obstacle_array = np.ma.masked_where(~obstacle_mask, np.ones_like(self.grid))
        plt.imshow(obstacle_array, cmap='binary', alpha=0.7, interpolation='nearest')
        
        # Plot start and goal
        if start:
            plt.plot(start[1], start[0], 'go', markersize=10, label='Start')
        if goal:
            plt.plot(goal[1], goal[0], 'ro', markersize=10, label='Goal')
        
        # Plot paths
        if standard_path:
            # Plot standard path
            std_x = [p[1] for p in standard_path]
            std_y = [p[0] for p in standard_path]
            plt.plot(std_x, std_y, 'b-', linewidth=2, label='Standard A*')
        
        if battery_path:
            # Plot battery-aware path
            bat_x = [p[1] for p in battery_path]
            bat_y = [p[0] for p in battery_path]
            plt.plot(bat_x, bat_y, 'g-', linewidth=2, label='Battery-Aware A*')
        
        # Add color bar and labels
        cbar = plt.colorbar(im, label='Elevation')
        
        # Add custom legend items
        from matplotlib.patches import Patch
        from matplotlib.lines import Line2D
        
        legend_elements = [
            Patch(facecolor='black', edgecolor='black', alpha=0.7, label='Buildings/Obstacles'),
            Line2D([0], [0], color='blue', lw=2, label='Standard A*'),
            Line2D([0], [0], color='green', lw=2, label='Battery-Aware A*')
        ]
        
        plt.legend(handles=legend_elements, loc='upper right')
        
        # Add title
        plt.title(f'City Map with Elevation (Size: {self.size}x{self.size})', fontsize=12)
        
        plt.tight_layout()
        
        if filename:
            plt.savefig(filename, dpi=150)
        
        plt.close()  # Close the figure instead of showing it
        
        return