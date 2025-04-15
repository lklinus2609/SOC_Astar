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

def battery_aware_astar(map_data, start, goal, battery):
    """Battery-aware A* algorithm"""
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
                
                # Update f_score with heuristic
                f_score[neighbor] = tentative_g_score + euclidean_distance(neighbor, goal)
                
                if neighbor not in open_set_hash:
                    counter += 1
                    open_set.put((f_score[neighbor], counter, neighbor))
                    open_set_hash.add(neighbor)
    
    # No path found
    return None, float('inf'), 0