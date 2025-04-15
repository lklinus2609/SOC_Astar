# Battery State Aware A* Simulation

This project demonstrates the performance difference between a battery state aware A* algorithm and a standard A* algorithm for robot pathfinding in environments with varying elevation.

## Project Overview

The simulation compares two pathfinding algorithms:
1. **Standard A*** - Uses Euclidean distance as the cost function, finding the shortest path without considering energy consumption.
2. **Battery-Aware A*** - Considers energy consumption due to elevation changes (inclines, declines) and terrain in its cost function.

The main hypothesis is that as map size increases, the battery-aware algorithm will achieve longer operating ranges by choosing more energy-efficient paths rather than simply the shortest paths. This is particularly relevant for robots operating in environments with significant elevation changes, where battery management is crucial for extended operation.

## Features

- City-like map generation with variable road patterns, buildings, and elevation features
- Realistic LiFePO4 battery model with:
  - Discharge efficiency varying with state of charge
  - Regenerative braking on downhill segments
  - Physics-based energy consumption calculation
- Comprehensive terrain representation with uphill, downhill, and flat segments
- Detailed performance metrics and visualization
- Side-by-side comparison of pathfinding algorithms 
- Analysis of terrain choices and energy efficiency
- Critical case identification where battery-aware algorithm succeeds but standard algorithm fails

## Project Structure

- `map.py` - City-style map generation with elevation features
- `battery.py` - Physics-based LiFePO4 battery model with regenerative braking
- `astar.py` - Standard and battery-aware A* algorithm implementations
- `simulation.py` - Simulation engine and path analysis tools
- `visualization.py` - Result visualization functions
- `main.py` - Simulation execution and enhanced performance reporting
- `requirements.txt` - Required Python packages

## How It Works

1. **Map Generation**: Creates city-like environments with roads, buildings, and varying elevations including strategic hills and valleys.
2. **Battery Modeling**: Models energy consumption based on physical principles, incorporating:
   - Gravitational work against elevation changes
   - Terrain-specific energy factors 
   - Regenerative energy recovery on downhill segments
   - State-of-charge dependent efficiency
3. **Path Planning**:
   - Standard A* focuses solely on minimizing distance
   - Battery-aware A* optimizes for energy consumption
4. **Analysis**: Detailed metrics on path characteristics, energy usage, and terrain preferences

## Simulation Results

The simulation identifies key differences between the algorithms:
- Battery-aware A* consistently uses less energy on larger maps
- Path length differences between the algorithms increase with map complexity
- Battery-aware A* makes strategic terrain choices, often opting for paths with:
  - More downhill segments to leverage regenerative braking
  - Fewer steep uphill segments to conserve energy
  - Sometimes longer but more energy-efficient routes

Results are stored in the `results` directory with:
- Performance comparison charts
- Path visualizations highlighting battery depletion points
- Detailed terrain and energy usage metrics
- Critical case visualizations where only battery-aware A* succeeds

## Usage

Run the simulation with:
```
python main.py
```

You can customize the simulation in `main.py`:
- Map sizes array (smaller sizes for quick testing, larger for more prominent differences)
- Number of trials per map size
- Battery capacity (adjust to make energy constraints more or less stringent)

## Enhanced Analytics

The simulation provides comprehensive analytics including:
- Success rates by map size
- Energy efficiency comparisons (distance per energy unit)
- Terrain distribution analysis
- Path characteristic differences
- Critical case identification and analysis
- Key findings summary

## Research Applications

This simulation demonstrates concepts applicable to:
- Autonomous robot navigation in urban or varying terrain environments
- Energy-aware routing for electric vehicles
- Path planning for drones and other battery-constrained mobile robots
- Studying the tradeoffs between distance optimization and energy optimization

## Requirements

- Python 3.6+
- NumPy
- Matplotlib
- Other dependencies in requirements.txt
