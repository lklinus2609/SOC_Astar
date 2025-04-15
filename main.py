import time
import os
from simulation import run_simulation
from visualization import create_performance_charts, create_summary_chart

def main():
    # Create output directory if it doesn't exist
    output_dir = "results"
    os.makedirs(output_dir, exist_ok=True)
    
    # Set simulation parameters with fewer map sizes for faster execution
    map_sizes = [80, 120, 160] 
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
    
    print("Simulation complete. Results saved to the 'results' directory.")
    print("Check the 'results' directory for detailed visualizations.")

if __name__ == "__main__":
    main()
