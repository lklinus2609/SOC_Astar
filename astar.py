import math
from queue import PriorityQueue

def euclidean_distance(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def standard_astar(map_data, start, goal):
    """Standard A* algorithm using Euclidean distance"""
    open_set = PriorityQueue()
    open_set.put((0, 0, start))  # (f_score, counter, position)
    
    # For tie-breaking in priority queue
    counter = 0
    
    # For path reconstruction
    came_from = {}
    
    # Cost from start to current node
    g_score = {start: 0}
    
    # Estimated total cost from start to goal through current node
    f_score = {start: euclidean_distance(start, goal)}
    
    # For tracking explored nodes
    open_set_hash = {start}
    
    while not open_set.empty():
        _, _, current = open_set.get()
        open_set_hash.remove(current)
        
        # If goal reached, reconstruct path
        if current == goal:
            path = []
            total_distance = g_score[current]
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, total_distance
        
        # Explore neighbors
        for neighbor in map_data.get_neighbors(current[0], current[1]):
            # Calculate distance to neighbor
            distance = euclidean_distance(current, neighbor)
            
            # Tentative g_score
            tentative_g_score = g_score[current] + distance
            
            # If we found a better path to the neighbor
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + euclidean_distance(neighbor, goal)
                
                if neighbor not in open_set_hash:
                    counter += 1
                    open_set.put((f_score[neighbor], counter, neighbor))
                    open_set_hash.add(neighbor)
    
    # No path found
    return None, float('inf')

def energy_heuristic(map_data, current, goal, battery_state):
    """
    Battery-aware energy heuristic function that estimates the minimum energy required
    to reach the goal from the current position.
    
    This heuristic considers:
    - Direct distance to goal
    - Elevation difference between current position and goal
    - Current battery state
    
    Returns an admissible (never overestimating) energy estimate.
    """
    # Calculate direct Euclidean distance to goal
    direct_distance = euclidean_distance(current, goal)
    
    # Get current and goal elevations
    current_elevation = map_data.elevation[current[0]][current[1]]
    goal_elevation = map_data.elevation[goal[0]][goal[1]]
    
    # Calculate elevation difference
    elevation_diff = goal_elevation - current_elevation
    
    # Base energy consumption estimate (flat terrain)
    base_consumption = 1.0 * direct_distance
    
    # Get current state of charge and efficiency
    current_soc = battery_state.get_state_of_charge()
    current_efficiency = battery_state.get_efficiency_at_soc(current_soc)
    
    # Estimate energy for elevation change
    # This is an optimistic estimate that considers:
    # 1. Uphill segments will require additional energy
    # 2. Downhill segments could potentially regenerate energy
    if elevation_diff > 0:  # Going uphill overall
        # Calculate optimistic slope angle
        slope_rad = math.atan2(elevation_diff, direct_distance)
        
        # Calculate gravitational force component
        gravitational_component = battery_state.robot_mass * 9.81 * math.sin(slope_rad)
        
        # Energy for elevation change (minimum required work against gravity)
        elevation_energy = gravitational_component * direct_distance / current_efficiency
        total_energy = base_consumption + max(0, elevation_energy)
    else:  # Flat or downhill overall
        # For downhill, we'll be optimistic and assume some energy recovery
        # The actual path may not be able to utilize all potential recovery
        # but the heuristic should be optimistic to remain admissible
        slope_rad = math.atan2(elevation_diff, direct_distance)
        gravitational_component = battery_state.robot_mass * 9.81 * math.sin(slope_rad)
        
        # Estimate potential energy recovery (with regeneration efficiency)
        recovery = max(0, -gravitational_component * direct_distance * battery_state.regen_efficiency)
        
        # Net energy consumption with optimistic recovery
        total_energy = max(0, base_consumption - recovery)
    
    # Ensure the heuristic never overestimates (must be admissible)
    # The multiplier here is a safety factor; it can be tuned 
    # but must be <= 1.0 to ensure admissibility
    admissibility_factor = 0.9
    
    return total_energy * admissibility_factor

def battery_aware_astar(map_data, start, goal, battery):
    """
    Battery-aware A* algorithm using a physics-based energy heuristic.
    
    This version uses energy as the consistent unit for both:
    - g(n): Actual energy consumed from start to current node
    - h(n): Estimated minimum energy required to reach the goal
    
    This creates a consistent, energy-optimizing pathfinding algorithm
    that considers battery state and elevation changes using pure physics principles.
    Energy calculations account for:
    - Base movement energy consumption
    - Work against gravity for uphill segments
    - Potential energy recovery for downhill segments
    - Battery efficiency based on state of charge
    """
    open_set = PriorityQueue()
    open_set.put((0, 0, start))  # (f_score, counter, position)
    
    # For tie-breaking in priority queue
    counter = 0
    
    # For path reconstruction
    came_from = {}
    
    # Cost from start to current node (energy consumed)
    g_score = {start: 0}
    
    # Battery state at each node
    battery_state = {start: battery.clone()}
    
    # Calculate initial f_score using the energy-based heuristic
    initial_estimate = energy_heuristic(map_data, start, goal, battery_state[start])
    f_score = {start: initial_estimate}
    
    # For tracking explored nodes
    open_set_hash = {start}
    
    while not open_set.empty():
        _, _, current = open_set.get()
        open_set_hash.remove(current)
        
        # If goal reached, reconstruct path
        if current == goal:
            path = []
            total_energy = g_score[current]
            remaining_charge = battery_state[current].get_state_of_charge()
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, total_energy, remaining_charge
        
        # Explore neighbors
        for neighbor in map_data.get_neighbors(current[0], current[1]):
            # Calculate energy consumption to neighbor
            current_elevation = map_data.elevation[current[0]][current[1]]
            next_elevation = map_data.elevation[neighbor[0]][neighbor[1]]
            distance = euclidean_distance(current, neighbor)
            
            # Get terrain type if available
            terrain_type = None
            if hasattr(map_data, 'terrain_type'):
                terrain_type = map_data.terrain_type[current[0]][current[1]]
            
            # Get energy consumption
            energy_consumption = battery_state[current].calculate_consumption(
                current_elevation, next_elevation, distance, terrain_type)
            
            # Check if we can go to this neighbor with current battery
            if not battery_state[current].can_traverse(current_elevation, next_elevation, distance, terrain_type):
                continue
            
            # Tentative g_score (energy consumed)
            tentative_g_score = g_score[current] + energy_consumption
            
            # If we found a better path to the neighbor
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                # Update path and scores
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                
                # Clone battery and update its state
                new_battery = battery_state[current].clone()
                new_battery.update_state(energy_consumption)
                battery_state[neighbor] = new_battery
                
                # Update f_score using the energy-based heuristic
                # Both g_score and heuristic are now in the same units (energy)
                energy_estimate = energy_heuristic(map_data, neighbor, goal, battery_state[neighbor])
                f_score[neighbor] = tentative_g_score + energy_estimate
                
                if neighbor not in open_set_hash:
                    counter += 1
                    open_set.put((f_score[neighbor], counter, neighbor))
                    open_set_hash.add(neighbor)
    
    # No path found
    return None, float('inf'), 0