import time
import os
from simulation import run_simulation
from visualization import create_performance_charts, create_summary_chart
from map import Map
from astar import standard_astar, battery_aware_astar
from battery import LiFePO4Battery

def test_improved_heuristic():
    """Run a simple test to verify the improved energy-based heuristic"""
    from map import Map
    from battery import LiFePO4Battery
    from astar import standard_astar, battery_aware_astar
    
    print("\nTesting improved energy-based heuristic...")
    print("====================================")
    
    # Create a small test map
    map_size = 50
    test_map = Map(map_size)
    test_map.generate_city_map(elevation_variance=0.7)  # Use higher elevation variance for clear differences
    
    # Get start and goal positions
    start = test_map.start_point
    goal = test_map.end_point
    
    print(f"Map size: {map_size}x{map_size}")
    print(f"Start: {start}, Goal: {goal}")
    
    # Run standard A*
    print("Running standard A*...")
    standard_path, standard_distance = standard_astar(test_map, start, goal)
    
    if standard_path:
        print(f"Standard A* found path with {len(standard_path)} nodes and distance {standard_distance:.2f}")
    else:
        print("Standard A* found no path")
    
    # Test with different battery capacities to showcase the difference
    for capacity in [200, 300, 400]:
        print(f"\nTesting with battery capacity: {capacity} Wh")
        
        # Run battery-aware A* with improved heuristic
        print("Running battery-aware A* with energy-based heuristic...")
        battery = LiFePO4Battery(capacity)
        battery_path, battery_energy, battery_remaining = battery_aware_astar(
            test_map, start, goal, battery)
        
        if battery_path:
            print(f"Battery-aware A* found path with {len(battery_path)} nodes and energy {battery_energy:.2f}")
            print(f"Remaining battery charge: {battery_remaining*100:.2f}%")
            
            # Compare paths if both algorithms found a path
            if standard_path:
                # Run simulation on standard path to get energy consumption
                from simulation import simulate_path
                standard_battery = LiFePO4Battery(capacity)
                standard_success, standard_remaining, _ = simulate_path(
                    test_map, standard_path, standard_battery)
                standard_energy = capacity - standard_remaining * capacity
                
                print("\nComparison:")
                print(f"Standard A*: {len(standard_path)} nodes, distance: {standard_distance:.2f}, energy: {standard_energy:.2f}, remaining: {standard_remaining*100:.2f}%")
                print(f"Battery A*: {len(battery_path)} nodes, energy: {battery_energy:.2f}, remaining: {battery_remaining*100:.2f}%")
                
                energy_diff = ((standard_energy - battery_energy) / standard_energy * 100) if standard_energy > 0 else 0
                print(f"Energy savings with battery-aware A*: {energy_diff:.2f}%")
                
                # Visualize the comparison
                output_dir = "results"
                os.makedirs(output_dir, exist_ok=True)
                test_map.visualize(start, goal, standard_path, battery_path, 
                                 f"{output_dir}/test_comparison_c{capacity}.png")
        else:
            print("Battery-aware A* found no path with this battery capacity")
    
    print("\nTest completed. Check the 'results' directory for visualizations.")

def test_physics_model():
    """Test the physics-based model without terrain factors"""
    print("\nTesting physics-based battery model (no terrain factors)...")
    print("===================================================")
    
    # Create a small test map with higher elevation variance
    map_size = 50
    test_map = Map(map_size)
    test_map.generate_city_map(elevation_variance=0.7)  # Higher variance for clearer results
    
    # Get start and goal positions
    start = test_map.start_point
    goal = test_map.end_point
    
    print(f"Map size: {map_size}x{map_size}")
    print(f"Start: {start}, Goal: {goal}")
    
    # Run standard A*
    print("Running standard A*...")
    standard_path, standard_distance = standard_astar(test_map, start, goal)
    
    if standard_path:
        print(f"Standard A* found path with {len(standard_path)} nodes and distance {standard_distance:.2f}")
    else:
        print("Standard A* found no path")
    
    # Test with battery capacity that should work for both algorithms
    capacity = 350
    print(f"\nTesting with battery capacity: {capacity} Wh")
    
    # Run battery-aware A* with improved heuristic
    print("Running battery-aware A* with physics-based model...")
    battery = LiFePO4Battery(capacity)
    battery_path, battery_energy, battery_remaining = battery_aware_astar(
        test_map, start, goal, battery)
    
    if battery_path:
        print(f"Battery-aware A* found path with {len(battery_path)} nodes and energy {battery_energy:.2f}")
        print(f"Remaining battery charge: {battery_remaining*100:.2f}%")
        
        # Compare paths if both algorithms found a path
        if standard_path:
            # Run simulation on standard path to get energy consumption
            from simulation import simulate_path
            standard_battery = LiFePO4Battery(capacity)
            standard_success, standard_remaining, _ = simulate_path(
                test_map, standard_path, standard_battery)
            standard_energy = capacity - standard_remaining * capacity
            
            print("\nComparison:")
            print(f"Standard A*: {len(standard_path)} nodes, distance: {standard_distance:.2f}, energy: {standard_energy:.2f}, remaining: {standard_remaining*100:.2f}%")
            print(f"Battery A*: {len(battery_path)} nodes, energy: {battery_energy:.2f}, remaining: {battery_remaining*100:.2f}%")
            
            energy_diff = ((standard_energy - battery_energy) / standard_energy * 100) if standard_energy > 0 else 0
            print(f"Energy savings with physics-based model: {energy_diff:.2f}%")
            
            # Visualize the comparison
            output_dir = "results"
            os.makedirs(output_dir, exist_ok=True)
            test_map.visualize(start, goal, standard_path, battery_path, 
                             f"{output_dir}/physics_model_comparison.png")
    else:
        print("Battery-aware A* found no path with this battery capacity")
    
    print("\nPhysics model test completed. Check the 'results' directory for visualizations.")

def main():
    # Create output directory if it doesn't exist
    output_dir = "results"
    os.makedirs(output_dir, exist_ok=True)
    
    # Test the improved heuristic first
    test_improved_heuristic()
    
    # Test the physics-based model (without terrain factors)
    test_physics_model()
    
    # Set simulation parameters with fewer map sizes for faster execution
    map_sizes = [80, 120, 400] 
    trials_per_size = 1 
    battery_capacity = 500
    
    print("Enhanced Battery-Aware A* vs Standard A* Simulation")
    print("===================================================")
    print(f"Map sizes: {map_sizes}")
    print(f"Trials per size: {trials_per_size}")
    print(f"Battery capacity: {battery_capacity} Wh")
    print(f"Using enhanced city-like map with terrain indicators")
    print(f"Using physics-based battery model with safety threshold")
    
    # Record start time
    start_time = time.time()
    
    # Run simulations
    results = run_simulation(map_sizes, battery_capacity, trials_per_size)
    
    # Record end time
    end_time = time.time()
    print(f"Simulation completed in {end_time - start_time:.2f} seconds")
    
    # Create performance charts
    print("Creating performance charts...")
    avg_data = create_performance_charts(results, os.path.join(output_dir, "performance_comparison.png"))
    
    # Create a summary chart of the findings
    print("Creating summary chart...")
    create_summary_chart(avg_data, os.path.join(output_dir, "summary_chart.png"))
    
    # Look for critical cases in results
    critical_cases = [r for r in results if r['battery_success'] and not r['standard_success']]
    if critical_cases:
        print(f"Found {len(critical_cases)} cases where battery-aware A* succeeded but standard A* failed")
    else:
        print("No critical case found where battery-aware A* succeeded but standard A* failed")
    
    # Generate enhanced summary statistics
    print("\nEnhanced Performance Summary:")
    print("============================")
    generate_enhanced_summary(results, map_sizes)
    
    print("\nSimulation complete. Results saved to the 'results' directory.")
    print("Check the 'results' directory for detailed visualizations.")

def generate_enhanced_summary(results, map_sizes):
    """Generate enhanced summary statistics for algorithm comparison"""
    # Calculate statistics for each map size
    size_stats = {}
    for size in map_sizes:
        size_results = [r for r in results if r['map_size'] == size]
        if not size_results:
            continue
            
        # Success rates
        std_success_rate = sum(1 for r in size_results if r['standard_success']) / len(size_results)
        bat_success_rate = sum(1 for r in size_results if r['battery_success']) / len(size_results)
        
        # Filter successful results for each algorithm
        std_successful = [r for r in size_results if r['standard_success']]
        bat_successful = [r for r in size_results if r['battery_success']]
        
        # Calculate metrics for successful runs
        if std_successful:
            avg_std_distance = sum(r['standard_distance'] for r in std_successful) / len(std_successful)
            avg_std_energy = sum(r['standard_energy'] for r in std_successful) / len(std_successful)
            avg_std_efficiency = avg_std_distance / max(0.01, avg_std_energy)  # Distance per energy unit
        else:
            avg_std_distance = float('inf')
            avg_std_energy = float('inf')
            avg_std_efficiency = 0
            
        if bat_successful:
            # For battery paths, we need to calculate distance since it's not returned by the algorithm
            bat_distances = []
            for r in bat_successful:
                if r['battery_path']:
                    path_distance = 0
                    for i in range(len(r['battery_path'])-1):
                        curr = r['battery_path'][i]
                        next_pos = r['battery_path'][i+1]
                        path_distance += ((curr[0]-next_pos[0])**2 + (curr[1]-next_pos[1])**2)**0.5
                    bat_distances.append(path_distance)
            
            avg_bat_distance = sum(bat_distances) / len(bat_distances) if bat_distances else float('inf')
            avg_bat_energy = sum(r['battery_energy'] for r in bat_successful) / len(bat_successful)
            avg_bat_efficiency = avg_bat_distance / max(0.01, avg_bat_energy)  # Distance per energy unit
        else:
            avg_bat_distance = float('inf')
            avg_bat_energy = float('inf')
            avg_bat_efficiency = 0
        
        # Terrain analysis - only for cases where both succeeded
        both_successful = [r for r in size_results if r['standard_success'] and r['battery_success']]
        terrain_stats = {}
        if both_successful:
            # Analyze terrain choices
            std_uphill_total = sum(r['standard_analysis']['uphill_distance'] for r in both_successful)
            std_downhill_total = sum(r['standard_analysis']['downhill_distance'] for r in both_successful)
            std_flat_total = sum(r['standard_analysis']['flat_distance'] for r in both_successful)
            std_total_distance = std_uphill_total + std_downhill_total + std_flat_total
            
            bat_uphill_total = sum(r['battery_analysis']['uphill_distance'] for r in both_successful)
            bat_downhill_total = sum(r['battery_analysis']['downhill_distance'] for r in both_successful)
            bat_flat_total = sum(r['battery_analysis']['flat_distance'] for r in both_successful)
            bat_total_distance = bat_uphill_total + bat_downhill_total + bat_flat_total
            
            if std_total_distance > 0:
                terrain_stats['std_uphill_pct'] = std_uphill_total / std_total_distance * 100
                terrain_stats['std_downhill_pct'] = std_downhill_total / std_total_distance * 100
                terrain_stats['std_flat_pct'] = std_flat_total / std_total_distance * 100
            else:
                terrain_stats['std_uphill_pct'] = 0
                terrain_stats['std_downhill_pct'] = 0
                terrain_stats['std_flat_pct'] = 0
                
            if bat_total_distance > 0:
                terrain_stats['bat_uphill_pct'] = bat_uphill_total / bat_total_distance * 100
                terrain_stats['bat_downhill_pct'] = bat_downhill_total / bat_total_distance * 100
                terrain_stats['bat_flat_pct'] = bat_flat_total / bat_total_distance * 100
            else:
                terrain_stats['bat_uphill_pct'] = 0
                terrain_stats['bat_downhill_pct'] = 0
                terrain_stats['bat_flat_pct'] = 0
            
        # Store stats for this map size
        size_stats[size] = {
            'std_success_rate': std_success_rate,
            'bat_success_rate': bat_success_rate,
            'avg_std_distance': avg_std_distance,
            'avg_std_energy': avg_std_energy,
            'std_efficiency': avg_std_efficiency,
            'avg_bat_distance': avg_bat_distance,
            'avg_bat_energy': avg_bat_energy,
            'bat_efficiency': avg_bat_efficiency,
            'terrain_stats': terrain_stats
        }
    
    # Print summary for each map size
    for size in sorted(size_stats.keys()):
        stats = size_stats[size]
        print(f"\nMap Size: {size}x{size}")
        print(f"  Success Rate: Standard A*: {stats['std_success_rate']*100:.1f}%, Battery-Aware A*: {stats['bat_success_rate']*100:.1f}%")
        
        if stats['std_success_rate'] > 0 and stats['bat_success_rate'] > 0:
            efficiency_diff = ((stats['bat_efficiency'] - stats['std_efficiency']) / max(0.01, stats['std_efficiency'])) * 100
            energy_diff = ((stats['avg_std_energy'] - stats['avg_bat_energy']) / max(0.01, stats['avg_std_energy'])) * 100
            distance_diff = ((stats['avg_bat_distance'] - stats['avg_std_distance']) / max(0.01, stats['avg_std_distance'])) * 100
            
            print(f"  Path Metrics:")
            print(f"    Average Path Distance: Standard: {stats['avg_std_distance']:.2f}, Battery-Aware: {stats['avg_bat_distance']:.2f} ({distance_diff:.1f}% diff)")
            print(f"    Average Energy Usage: Standard: {stats['avg_std_energy']:.2f}, Battery-Aware: {stats['avg_bat_energy']:.2f} ({energy_diff:.1f}% diff)")
            print(f"    Energy Efficiency: Standard: {stats['std_efficiency']:.4f}, Battery-Aware: {stats['bat_efficiency']:.4f} ({efficiency_diff:.1f}% improvement)")
            
            # Only print terrain stats if we have them
            if stats['terrain_stats']:
                print(f"  Terrain Usage:")
                print(f"    Standard A*: Uphill: {stats['terrain_stats']['std_uphill_pct']:.1f}%, Downhill: {stats['terrain_stats']['std_downhill_pct']:.1f}%, Flat: {stats['terrain_stats']['std_flat_pct']:.1f}%")
                print(f"    Battery A*: Uphill: {stats['terrain_stats']['bat_uphill_pct']:.1f}%, Downhill: {stats['terrain_stats']['bat_downhill_pct']:.1f}%, Flat: {stats['terrain_stats']['bat_flat_pct']:.1f}%")
    
    # Print overall summary
    print("\nOverall Performance Summary:")
    
    # Calculate overall statistics
    all_std_success = sum(1 for r in results if r['standard_success'])
    all_bat_success = sum(1 for r in results if r['battery_success'])
    total_trials = len(results)
    
    # Calculate overall success rates
    overall_std_success_rate = all_std_success / total_trials if total_trials > 0 else 0
    overall_bat_success_rate = all_bat_success / total_trials if total_trials > 0 else 0
    
    print(f"  Total Trials: {total_trials}")
    print(f"  Overall Success Rate: Standard A*: {overall_std_success_rate*100:.1f}%, Battery-Aware A*: {overall_bat_success_rate*100:.1f}%")
    
    # Analyze critical cases where battery-aware A* succeeded but standard A* failed
    critical_cases = [r for r in results if r['battery_success'] and not r['standard_success']]
    if critical_cases:
        avg_energy_saved = sum(r['battery_remaining'] * 100 for r in critical_cases) / len(critical_cases)
        print(f"  Critical Cases: {len(critical_cases)} instances where Battery-Aware A* succeeded but Standard A* failed")
        print(f"  Average Energy Saved: {avg_energy_saved:.2f}% of battery capacity")
    
    # Key findings summary
    print("\nKey Findings:")
    
    if overall_bat_success_rate > overall_std_success_rate:
        print("  - Battery-aware A* demonstrated higher overall success rate")
    
    # Check if there are consistent energy efficiency improvements in larger maps
    large_maps = [size for size in size_stats.keys() if size >= 80]  # Define large maps as ≥ 80x80
    if large_maps:
        large_map_efficiency_improvement = all(size_stats[size]['bat_efficiency'] > size_stats[size]['std_efficiency'] for size in large_maps if size_stats[size]['std_efficiency'] > 0 and size_stats[size]['bat_efficiency'] > 0)
        if large_map_efficiency_improvement:
            print("  - Battery-aware A* consistently showed higher energy efficiency in larger maps")
    
    # Check if terrain usage patterns are significantly different
    all_terrain_stats = [stats['terrain_stats'] for stats in size_stats.values() if 'terrain_stats' in stats and stats['terrain_stats']]
    if all_terrain_stats:
        uphill_difference = sum(stats['std_uphill_pct'] - stats['bat_uphill_pct'] for stats in all_terrain_stats) / len(all_terrain_stats)
        if abs(uphill_difference) > 5:  # 5% threshold for significance
            if uphill_difference > 0:
                print(f"  - Battery-aware A* consistently chose routes with {abs(uphill_difference):.1f}% less uphill terrain")
            else:
                print(f"  - Battery-aware A* surprisingly chose routes with {abs(uphill_difference):.1f}% more uphill terrain")

if __name__ == "__main__":
    main()
