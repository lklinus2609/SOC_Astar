import matplotlib.pyplot as plt
import numpy as np
from astar import euclidean_distance
from battery import LiFePO4Battery

def create_performance_charts(results, output_filename="performance_comparison.png"):
    """Create charts to visualize performance metrics"""
    map_sizes = sorted(list(set([r['map_size'] for r in results])))
    
    # Organize data by map size
    size_data = {size: [] for size in map_sizes}
    for result in results:
        size_data[result['map_size']].append(result)
    
    # Calculate averages
    avg_data = []
    for size, trials in size_data.items():
        # Count successes
        std_success_count = sum(1 for t in trials if t['standard_success'])
        bat_success_count = sum(1 for t in trials if t['battery_success'])
        
        # Calculate averages for successful runs
        std_successful = [t for t in trials if t['standard_success']]
        bat_successful = [t for t in trials if t['battery_success']]
        
        avg_std_distance = np.mean([t['standard_distance'] for t in std_successful]) if std_successful else float('inf')
        avg_std_energy = np.mean([t['standard_energy'] for t in std_successful]) if std_successful else float('inf')
        avg_std_remaining = np.mean([t['standard_remaining'] for t in std_successful]) if std_successful else 0
        
        avg_bat_distance = np.mean([euclidean_distance(t['start'], t['goal']) for t in bat_successful]) if bat_successful else float('inf')
        avg_bat_energy = np.mean([t['battery_energy'] for t in bat_successful]) if bat_successful else float('inf')
        avg_bat_remaining = np.mean([t['battery_remaining'] for t in bat_successful]) if bat_successful else 0
        
        # Calculate execution time
        avg_std_time = np.mean([t['standard_time'] for t in trials]) if 'standard_time' in trials[0] else 0
        avg_bat_time = np.mean([t['battery_time'] for t in trials]) if 'battery_time' in trials[0] else 0
        
        avg_data.append({
            'map_size': size,
            'std_success_rate': std_success_count / len(trials) if trials else 0,
            'bat_success_rate': bat_success_count / len(trials) if trials else 0,
            'avg_std_distance': avg_std_distance,
            'avg_std_energy': avg_std_energy,
            'avg_std_remaining': avg_std_remaining,
            'avg_std_time': avg_std_time,
            'avg_bat_distance': avg_bat_distance,
            'avg_bat_energy': avg_bat_energy,
            'avg_bat_remaining': avg_bat_remaining,
            'avg_bat_time': avg_bat_time
        })
    
    # Create a figure with 2x2 subplots
    plt.figure(figsize=(15, 12))
    
    # Plot success rates
    plt.subplot(2, 2, 1)
    plt.plot([d['map_size'] for d in avg_data], [d['std_success_rate'] for d in avg_data], 'b-o', label='Standard A*')
    plt.plot([d['map_size'] for d in avg_data], [d['bat_success_rate'] for d in avg_data], 'g-o', label='Battery-Aware A*')
    plt.title('Success Rate by Map Size')
    plt.xlabel('Map Size')
    plt.ylabel('Success Rate')
    plt.legend()
    plt.grid(True)
    
    # Plot average energy consumption
    plt.subplot(2, 2, 2)
    plt.plot([d['map_size'] for d in avg_data], [d['avg_std_energy'] for d in avg_data], 'b-o', label='Standard A*')
    plt.plot([d['map_size'] for d in avg_data], [d['avg_bat_energy'] for d in avg_data], 'g-o', label='Battery-Aware A*')
    plt.title('Average Energy Consumption by Map Size')
    plt.xlabel('Map Size')
    plt.ylabel('Energy Consumed')
    plt.legend()
    plt.grid(True)
    
    # Plot average remaining battery
    plt.subplot(2, 2, 3)
    plt.plot([d['map_size'] for d in avg_data], [d['avg_std_remaining'] for d in avg_data], 'b-o', label='Standard A*')
    plt.plot([d['map_size'] for d in avg_data], [d['avg_bat_remaining'] for d in avg_data], 'g-o', label='Battery-Aware A*')
    plt.title('Average Remaining Battery')
    plt.xlabel('Map Size')
    plt.ylabel('Remaining Battery (percentage)')
    plt.legend()
    plt.grid(True)
    
    # Plot execution time
    plt.subplot(2, 2, 4)
    plt.plot([d['map_size'] for d in avg_data], [d['avg_std_time'] for d in avg_data], 'b-o', label='Standard A*')
    plt.plot([d['map_size'] for d in avg_data], [d['avg_bat_time'] for d in avg_data], 'g-o', label='Battery-Aware A*')
    plt.title('Average Execution Time')
    plt.xlabel('Map Size')
    plt.ylabel('Time (seconds)')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(output_filename, dpi=150)
    plt.close()  # Close the figure instead of showing it
    
    return avg_data

def create_summary_chart(avg_data, output_filename="summary_chart.png"):
    """Create a comprehensive summary chart of the findings"""
    # Extract the data for the largest map size that has successful runs
    # This gives us the most dramatic comparison
    valid_data = [d for d in avg_data if d['bat_success_rate'] > 0]
    if not valid_data:
        print("No valid data for summary chart")
        return
    
    largest_map_data = max(valid_data, key=lambda x: x['map_size'])
    map_size = largest_map_data['map_size']
    
    # Calculate energy efficiency for both algorithms
    std_efficiency = largest_map_data['avg_std_distance'] / max(0.01, largest_map_data['avg_std_energy']) if largest_map_data['std_success_rate'] > 0 else 0
    bat_efficiency = largest_map_data['avg_bat_distance'] / max(0.01, largest_map_data['avg_bat_energy'])
    
    # Calculate improvement percentages
    success_improvement = ((largest_map_data['bat_success_rate'] - largest_map_data['std_success_rate']) / 
                         max(0.01, largest_map_data['std_success_rate'])) * 100
    battery_improvement = ((largest_map_data['avg_bat_remaining'] - largest_map_data['avg_std_remaining']) / 
                          max(0.01, largest_map_data['avg_std_remaining'])) * 100
    efficiency_improvement = ((bat_efficiency - std_efficiency) / 
                             max(0.01, std_efficiency)) * 100
    
    # Create figure with two subplots side by side
    fig = plt.figure(figsize=(20, 10))
    
    # First subplot - Bar chart comparing key metrics
    ax1 = fig.add_subplot(121)
    
    # Create metrics for comparison
    metrics = [
        ('Success Rate', largest_map_data['std_success_rate'], largest_map_data['bat_success_rate']),
        ('Remaining\nBattery', largest_map_data['avg_std_remaining'], largest_map_data['avg_bat_remaining']),
        ('Energy\nEfficiency', std_efficiency, bat_efficiency)
    ]
    
    # Create a bar chart
    labels = [m[0] for m in metrics]
    std_values = [m[1] for m in metrics]
    bat_values = [m[2] for m in metrics]
    
    x = np.arange(len(labels))
    width = 0.35
    
    rects1 = ax1.bar(x - width/2, std_values, width, label='Standard A*', color='blue', alpha=0.7)
    rects2 = ax1.bar(x + width/2, bat_values, width, label='Battery-Aware A*', color='green', alpha=0.7)
    
    # Add labels and title
    ax1.set_title(f'Performance Metrics Comparison (Map Size: {map_size}x{map_size})', fontsize=16)
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, fontsize=14)
    ax1.set_ylabel('Performance Metrics', fontsize=14)
    ax1.legend(fontsize=14)
    ax1.grid(True, linestyle='--', alpha=0.3)
    
    # Add value labels on the bars
    def autolabel(rects):
        for rect in rects:
            height = rect.get_height()
            ax1.annotate(f'{height:.2f}',
                        xy=(rect.get_x() + rect.get_width()/2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom', fontsize=12)
    
    autolabel(rects1)
    autolabel(rects2)
    
    # Second subplot - Improvement percentage
    ax2 = fig.add_subplot(122)
    
    # Improvement metrics
    improvement_metrics = [
        ('Success Rate', success_improvement),
        ('Remaining\nBattery', battery_improvement),
        ('Energy\nEfficiency', efficiency_improvement)
    ]
    
    # Filter out metrics that don't make sense (e.g., if standard A* had zero success rate)
    valid_improvements = [(label, value) for label, value in improvement_metrics if np.isfinite(value)]
    
    if valid_improvements:
        labels = [m[0] for m in valid_improvements]
        values = [m[1] for m in valid_improvements]
        
        # Use a color gradient based on improvement percentage
        colors = ['#a1d99b' if v > 0 else '#e6550d' for v in values]
        
        y_pos = np.arange(len(labels))
        
        # Horizontal bar chart for improvement percentages
        bars = ax2.barh(y_pos, values, align='center', color=colors, alpha=0.7)
        ax2.set_yticks(y_pos)
        ax2.set_yticklabels(labels, fontsize=14)
        ax2.set_xlabel('Improvement Percentage (%)', fontsize=14)
        ax2.set_title('Performance Improvement with Battery-Aware A*', fontsize=16)
        ax2.grid(True, linestyle='--', alpha=0.3)
        
        # Add percentage labels
        for i, v in enumerate(values):
            if v > 0:
                ax2.text(v + 5, i, f"+{v:.1f}%", va='center', fontsize=12)
            else:
                ax2.text(v - 30, i, f"{v:.1f}%", va='center', fontsize=12)
        
        # Add a reference line at 0%
        ax2.axvline(x=0, color='black', linestyle='-', alpha=0.3)
    
    # Add a textbox with key findings
    findings_text = (
        f"Key Findings for {map_size}x{map_size} Maps:\n\n"
        f"1. Success Rate: Battery-aware A* achieves "
        f"{largest_map_data['bat_success_rate']*100:.1f}% success vs "
        f"{largest_map_data['std_success_rate']*100:.1f}% for standard A*\n\n"
        f"2. Energy Efficiency: Battery-aware A* is "
        f"{abs(efficiency_improvement):.1f}% {'more' if efficiency_improvement > 0 else 'less'} "
        f"energy efficient\n\n"
        f"3. Battery Conservation: Battery-aware A* maintains "
        f"{largest_map_data['avg_bat_remaining']*100:.1f}% charge vs "
        f"{largest_map_data['avg_std_remaining']*100:.1f}% for standard A*"
    )
    
    plt.figtext(0.5, 0.01, findings_text, ha="center", fontsize=14,
               bbox={"facecolor":"lightgray", "alpha":0.5, "pad":10}, 
               wrap=True, horizontalalignment='center')
    
    plt.tight_layout(rect=[0, 0.15, 1, 0.95])  # Adjust layout to make room for text
    plt.savefig(output_filename, dpi=150)
    plt.close()  # Close the figure instead of showing it
