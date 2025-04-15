import random
import numpy as np
import matplotlib.pyplot as plt
from battery import LiFePO4Battery
from astar import euclidean_distance, standard_astar, battery_aware_astar
import math
import time

def find_valid_start_goal(map_data, min_distance=None):
    """Find valid start and goal positions on the map"""
    # Check if map has predefined start and end points
    if hasattr(map_data, 'start_point') and hasattr(map_data, 'end_point'):
        return map_data.start_point, map_data.end_point
    
    # If no predefined points, generate them with minimum distance
    if min_distance is None:
        min_distance = map_data.size // 3  # Set minimum distance as 1/3 of map size
    
    valid_positions = []
    for i in range(map_data.size):
        for j in range(map_data.size):
            if map_data.is_valid_position(i, j):
                valid_positions.append((i, j))
    
    attempts = 0
    max_attempts = 100  # Avoid infinite loop
    
    while attempts < max_attempts:
        if len(valid_positions) < 2:
            raise ValueError("Not enough valid positions on map")
        
        start = random.choice(valid_positions)
        goal = random.choice(valid_positions)
        
        if start != goal and euclidean_distance(start, goal) >= min_distance:
            # Check if path exists (simple Manhattan check for quick filtering)
            manhattan_dist = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
            if manhattan_dist < map_data.size * 2:  # Reasonable path length
                return start, goal
        
        attempts += 1
    
    # If we couldn't find good positions, use corners
    return (0, 0), (map_data.size-1, map_data.size-1)

def simulate_path(map_data, path, battery):
    """
    Simulate following a path with the given battery
    Returns: (success, remaining_charge, failure_point)
    """
    if not path:
        return False, 0, None
    
    # Create a fresh battery copy for simulation
    sim_battery = battery.clone()
    energy_used = 0
    
    for i in range(len(path) - 1):
        current = path[i]
        next_pos = path[i + 1]
        
        current_elevation = map_data.elevation[current[0]][current[1]]
        next_elevation = map_data.elevation[next_pos[0]][next_pos[1]]
        distance = euclidean_distance(current, next_pos)
        
        # Get terrain type if available
        terrain_type = None
        if hasattr(map_data, 'terrain_type'):
            terrain_type = map_data.terrain_type[current[0]][current[1]]
        
        # Calculate energy consumption
        energy_consumption = sim_battery.calculate_consumption(
            current_elevation, next_elevation, distance, terrain_type)
        energy_used += energy_consumption
        
        # Update battery state
        if not sim_battery.update_state(energy_consumption):
            # Battery depleted - return the position where failure occurred
            return False, sim_battery.get_state_of_charge(), i
    
    return True, sim_battery.get_state_of_charge(), None  # Reached goal with remaining charge

def analyze_path_details(map_data, path, battery):
    """Generate detailed analysis of a path including energy usage for each segment"""
    if not path or len(path) < 2:
        return {}
    
    # Create a fresh battery copy for analysis
    sim_battery = battery.clone()
    
    # Initialize metrics
    distance_traveled = 0
    total_energy_consumption = 0
    uphill_energy = 0
    downhill_energy = 0
    flat_energy = 0
    uphill_distance = 0
    downhill_distance = 0
    flat_distance = 0
    segment_details = []
    battery_levels = [sim_battery.get_state_of_charge()]
    failure_index = None
    
    # Process each segment
    for i in range(len(path) - 1):
        current = path[i]
        next_pos = path[i + 1]
        
        # Get segment characteristics
        current_elevation = map_data.elevation[current[0]][current[1]]
        next_elevation = map_data.elevation[next_pos[0]][next_pos[1]]
        terrain_type = None
        if hasattr(map_data, 'terrain_type'):
            terrain_type = map_data.terrain_type[current[0]][current[1]]
        
        # Calculate segment metrics
        distance = euclidean_distance(current, next_pos)
        slope = map_data.calculate_slope(current, next_pos) if hasattr(map_data, 'calculate_slope') else 0
        energy_consumption = sim_battery.calculate_consumption(
            current_elevation, next_elevation, distance, terrain_type)
        
        # Update totals
        distance_traveled += distance
        total_energy_consumption += energy_consumption
        
        # Categorize segment by slope
        if slope > 2:  # Uphill (more than 2 degrees)
            uphill_energy += energy_consumption
            uphill_distance += distance
        elif slope < -2:  # Downhill (less than -2 degrees)
            downhill_energy += energy_consumption
            downhill_distance += distance
        else:  # Flat (-2 to 2 degrees)
            flat_energy += energy_consumption
            flat_distance += distance
        
        # Update battery
        if not sim_battery.update_state(energy_consumption):
            # Record the failure index if battery is depleted
            if failure_index is None:
                failure_index = i
        
        battery_levels.append(sim_battery.get_state_of_charge())
        
        # Store segment details
        segment_details.append({
            'segment': i,
            'start': current,
            'end': next_pos,
            'distance': distance,
            'elevation_change': next_elevation - current_elevation,
            'slope': slope,
            'energy_consumption': energy_consumption,
            'battery_soc': battery_levels[-1]
        })
    
    # Compile all metrics
    path_analysis = {
        'total_distance': distance_traveled,
        'total_energy': total_energy_consumption,
        'uphill_energy': uphill_energy,
        'downhill_energy': downhill_energy,
        'flat_energy': flat_energy,
        'uphill_distance': uphill_distance,
        'downhill_distance': downhill_distance,
        'flat_distance': flat_distance,
        'start_soc': battery_levels[0],
        'final_soc': battery_levels[-1],
        'battery_levels': battery_levels,
        'segment_details': segment_details,
        'failure_index': failure_index
    }
    
    return path_analysis

def visualize_comparison(map_data, start, goal, standard_path, battery_path, filename=None, failure_index=None):
    """Visualize both paths on the map with battery failure point if applicable"""
    plt.figure(figsize=(12, 10))
    
    # Create a custom colormap for different cell types
    # Create masked arrays for each type
    road_mask = (map_data.grid == 0)
    obstacle_mask = (map_data.grid == 1)
    
    # Create elevation map for roads only
    road_elevation = np.ma.masked_where(~road_mask, map_data.elevation)
    
    # Plot road elevation with terrain colormap
    im = plt.imshow(road_elevation, cmap='terrain', interpolation='nearest')
    
    # Plot buildings/obstacles
    obstacle_array = np.ma.masked_where(~obstacle_mask, np.ones_like(map_data.grid))
    plt.imshow(obstacle_array, cmap='binary', alpha=0.7, interpolation='nearest')
    
    # Plot start and goal
    if start:
        plt.plot(start[1], start[0], 'go', markersize=10, label='Start')
    if goal:
        plt.plot(goal[1], goal[0], 'ro', markersize=10, label='Goal')
    
    # Plot paths
    if standard_path:
        # Check if there's a failure point for the standard path
        if failure_index is not None and failure_index < len(standard_path) - 1:
            # Split the path into two parts: successful and failed
            valid_path = standard_path[:failure_index+1]
            invalid_path = standard_path[failure_index:]
            
            # Plot the valid part of the standard path
            valid_x = [p[1] for p in valid_path]
            valid_y = [p[0] for p in valid_path]
            plt.plot(valid_x, valid_y, 'b-', linewidth=2, label='Standard A*')
            
            # Plot the invalid part with different style (dashed red)
            invalid_x = [p[1] for p in invalid_path]
            invalid_y = [p[0] for p in invalid_path]
            plt.plot(invalid_x, invalid_y, 'r--', linewidth=2, alpha=0.7, label='Unreachable Path (Battery Depleted)')
            
            # Mark the failure point with a red X
            failure_point = standard_path[failure_index]
            plt.plot(failure_point[1], failure_point[0], 'rx', markersize=10, markeredgewidth=2)
            
            # Add a text annotation near the failure point
            plt.annotate('Battery Depleted', 
                        xy=(failure_point[1], failure_point[0]),
                        xytext=(failure_point[1]+5, failure_point[0]-5),
                        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="red", alpha=0.7),
                        arrowprops=dict(arrowstyle="->", connectionstyle="arc3", color="red"))
        else:
            # Plot the entire standard path normally
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
        Line2D([0], [0], color='blue', lw=2, label='Standard A*')
    ]
    
    # Add different legend elements if there's a failure point
    if failure_index is not None:
        legend_elements.append(Line2D([0], [0], color='red', lw=2, linestyle='--', alpha=0.7, 
                                     label='Unreachable Path (Battery Depleted)'))
        legend_elements.append(Line2D([0], [0], marker='x', color='red', lw=0, markersize=8,
                                     label='Battery Depletion Point'))
    
    legend_elements.append(Line2D([0], [0], color='green', lw=2, label='Battery-Aware A*'))
    
    plt.legend(handles=legend_elements, loc='upper right')
    
    # Add title
    plt.title(f'Path Comparison (Size: {map_data.size}x{map_data.size})', fontsize=12)
    
    plt.tight_layout()
    
    if filename:
        plt.savefig(filename, dpi=150)
    
    plt.close()  # Close the figure instead of showing it
    
    return

def visualize_path_metrics(std_analysis, bat_analysis, filename=None):
    """Visualize detailed path metrics comparison"""
    if not std_analysis or not bat_analysis:
        return
    
    plt.figure(figsize=(15, 10))
    
    # Create a 2x2 subplot layout
    plt.subplot(2, 2, 1)
    # Plot distance comparison
    labels = ['Total', 'Uphill', 'Flat', 'Downhill']
    std_distances = [std_analysis['total_distance'], 
                    std_analysis['uphill_distance'],
                    std_analysis['flat_distance'], 
                    std_analysis['downhill_distance']]
    bat_distances = [bat_analysis['total_distance'], 
                    bat_analysis['uphill_distance'],
                    bat_analysis['flat_distance'], 
                    bat_analysis['downhill_distance']]
    
    x = np.arange(len(labels))
    width = 0.35
    
    plt.bar(x - width/2, std_distances, width, label='Standard A*', color='blue')
    plt.bar(x + width/2, bat_distances, width, label='Battery-Aware A*', color='green')
    plt.xlabel('Terrain Type')
    plt.ylabel('Distance')
    plt.title('Distance Comparison by Terrain')
    plt.xticks(x, labels)
    plt.legend()
    
    # Plot energy consumption comparison
    plt.subplot(2, 2, 2)
    std_energy = [std_analysis['total_energy'], 
                 std_analysis['uphill_energy'],
                 std_analysis['flat_energy'], 
                 std_analysis['downhill_energy']]
    bat_energy = [bat_analysis['total_energy'], 
                 bat_analysis['uphill_energy'],
                 bat_analysis['flat_energy'], 
                 bat_analysis['downhill_energy']]
    
    plt.bar(x - width/2, std_energy, width, label='Standard A*', color='blue')
    plt.bar(x + width/2, bat_energy, width, label='Battery-Aware A*', color='green')
    plt.xlabel('Terrain Type')
    plt.ylabel('Energy Consumption')
    plt.title('Energy Consumption by Terrain')
    plt.xticks(x, labels)
    plt.legend()
    
    # Plot battery state of charge throughout path
    plt.subplot(2, 2, 3)
    
    # Standard path battery levels
    std_battery_levels = np.array(std_analysis['battery_levels']) * 100
    std_x = np.arange(len(std_battery_levels))
    plt.plot(std_x, std_battery_levels, 'b-', label='Standard A*')
    
    # If there's a failure index, mark it
    if 'failure_index' in std_analysis and std_analysis['failure_index'] is not None:
        failure_idx = std_analysis['failure_index']
        if failure_idx < len(std_battery_levels):
            plt.axvline(x=failure_idx, color='red', linestyle='--', alpha=0.5)
            plt.plot(failure_idx, std_battery_levels[failure_idx], 'rx', markersize=10)
            plt.annotate('Battery Depleted', 
                        xy=(failure_idx, std_battery_levels[failure_idx]),
                        xytext=(failure_idx+5, std_battery_levels[failure_idx]+10),
                        arrowprops=dict(arrowstyle="->", color="red"),
                        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="red", alpha=0.7))
    
    # Battery-aware path battery levels
    bat_battery_levels = np.array(bat_analysis['battery_levels']) * 100
    bat_x = np.arange(len(bat_battery_levels))
    plt.plot(bat_x, bat_battery_levels, 'g-', label='Battery-Aware A*')
    
    plt.xlabel('Path Progress (segments)')
    plt.ylabel('Battery State of Charge (%)')
    plt.title('Battery Levels Throughout Path')
    plt.grid(True)
    plt.legend()
    
    # Plot energy efficiency (distance per energy unit)
    plt.subplot(2, 2, 4)
    std_efficiency = std_analysis['total_distance'] / max(0.01, std_analysis['total_energy'])
    bat_efficiency = bat_analysis['total_distance'] / max(0.01, bat_analysis['total_energy'])
    
    plt.bar(['Standard A*', 'Battery-Aware A*'], [std_efficiency, bat_efficiency],
            color=['blue', 'green'])
    plt.ylabel('Distance per Energy Unit')
    plt.title('Energy Efficiency Comparison')
    
    plt.tight_layout()
    
    if filename:
        plt.savefig(filename, dpi=150)
    
    plt.close()  # Close the figure instead of showing it

def run_simulation(map_sizes, battery_capacity, trials_per_size):
    """Run simulation for different map sizes and compare algorithms"""
    results = []
    
    for size in map_sizes:
        print(f"Running simulations for map size {size}x{size}...")
        
        for trial in range(trials_per_size):
            print(f"  Trial {trial+1}/{trials_per_size}")
            
            # Generate city-like map with simplified parameters for faster generation
            from map import Map
            map_data = Map(size)
            
            # Use simpler city map generation parameters
            road_density = 0.25
            block_size = max(3, size//20)  # Smaller blocks for fewer calculations
            
            print(f"  Generating city map with size: {size}x{size}")
            start_time = time.time()
            map_data.generate_city_map(road_density=road_density, 
                                      block_size=block_size, 
                                      elevation_variance=0.3)
            print(f"  Map generation took {time.time() - start_time:.2f} seconds")
            
            # Get start and goal from the map (predefined with good distance)
            start, goal = find_valid_start_goal(map_data)
            print(f"  Start: {start}, Goal: {goal}")
            
            # Run standard A*
            print("  Running standard A*...")
            start_time = time.time()
            standard_path, standard_distance = standard_astar(map_data, start, goal)
            standard_time = time.time() - start_time
            print(f"  Standard A* took {standard_time:.2f} seconds")
            
            # Calculate energy needed for standard path
            standard_energy = 0
            standard_success = False
            standard_remaining = 0
            standard_analysis = None
            failure_index = None
            
            if standard_path:
                print(f"  Standard A* found path with {len(standard_path)} nodes, distance: {standard_distance:.2f}")
                standard_battery = LiFePO4Battery(battery_capacity)
                standard_success, standard_remaining, failure_index = simulate_path(
                    map_data, standard_path, standard_battery)
                standard_energy = battery_capacity - standard_remaining * battery_capacity
                
                # Generate detailed analysis
                standard_analysis = analyze_path_details(map_data, standard_path, 
                                                         LiFePO4Battery(battery_capacity))
                
                print(f"  Standard path success: {standard_success}, " +
                      f"energy used: {standard_energy:.2f}, " + 
                      f"battery remaining: {standard_remaining*100:.1f}%")
                if not standard_success and failure_index is not None:
                    print(f"  Battery depleted at segment {failure_index} of the path")
            else:
                print("  Standard A* found no path")
            
            # Run battery-aware A*
            print("  Running battery-aware A*...")
            start_time = time.time()
            battery = LiFePO4Battery(battery_capacity)
            battery_path, battery_energy, battery_remaining = battery_aware_astar(
                map_data, start, goal, battery)
            battery_time = time.time() - start_time
            print(f"  Battery-aware A* took {battery_time:.2f} seconds")
            
            # Determine if battery-aware path succeeded
            battery_success = battery_path is not None
            battery_analysis = None
            
            if battery_path:
                print(f"  Battery-aware A* found path with {len(battery_path)} nodes, " +
                      f"energy: {battery_energy:.2f}")
                
                # Generate detailed analysis
                battery_analysis = analyze_path_details(map_data, battery_path, 
                                                       LiFePO4Battery(battery_capacity))
                
                print(f"  Battery path success: {battery_success}, " +
                      f"battery remaining: {battery_remaining*100:.1f}%")
            else:
                print("  Battery-aware A* found no path")
            
            # Store results
            trial_result = {
                'map_size': size,
                'trial': trial,
                'standard_path': standard_path,
                'standard_distance': standard_distance if standard_path else float('inf'),
                'standard_energy': standard_energy,
                'standard_success': standard_success,
                'standard_remaining': standard_remaining,
                'standard_time': standard_time,
                'standard_analysis': standard_analysis,
                'failure_index': failure_index,
                'battery_path': battery_path,
                'battery_energy': battery_energy if battery_path else float('inf'),
                'battery_remaining': battery_remaining if battery_path else 0,
                'battery_success': battery_success,
                'battery_time': battery_time,
                'battery_analysis': battery_analysis,
                'map_data': map_data,
                'start': start,
                'goal': goal
            }
            
            results.append(trial_result)
            
            # Only visualize critical cases (battery succeeds but standard fails)
            if battery_success and not standard_success:
                print(f"Found critical case in map size {size}x{size}, trial {trial}")
                # Save visualization for critical case
                critical_filename = f"results/critical_case_size{size}_trial{trial}.png"
                visualize_comparison(map_data, start, goal, standard_path, battery_path, 
                                    critical_filename, failure_index)
                
                # Create metrics visualization for critical cases
                if standard_analysis and battery_analysis:
                    metrics_filename = f"results/metrics_size{size}_trial{trial}.png"
                    visualize_path_metrics(standard_analysis, battery_analysis, metrics_filename)
                
            # Stop if both algorithms fail (as per requirements)
            if not standard_success and not battery_success:
                print(f"Both algorithms failed in map size {size}x{size}, trial {trial}")
                break
    
    # Create visualizations for the largest map size
    largest_size = max(map_sizes)
    largest_size_results = [r for r in results if r['map_size'] == largest_size]
    
    # Always visualize at least one case from the largest map
    if largest_size_results and not any(r['battery_success'] and not r['standard_success'] for r in results):
        print("Creating visualization for largest map size...")
        result = largest_size_results[0]  # Take the first result from largest map
        
        # Create path visualization
        vis_filename = f"results/comparison_size{result['map_size']}_trial{result['trial']}.png"
        visualize_comparison(result['map_data'], result['start'], result['goal'], 
                          result['standard_path'], result['battery_path'], 
                          vis_filename, result.get('failure_index'))
        
        # Create metrics visualization
        if result['standard_analysis'] and result['battery_analysis']:
            metrics_filename = f"results/metrics_size{result['map_size']}_trial{result['trial']}.png"
            visualize_path_metrics(result['standard_analysis'], result['battery_analysis'], 
                                metrics_filename)

    return results
