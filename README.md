# Battery State Aware A* Simulation

This project demonstrates the performance difference between a battery state aware A* algorithm and a standard A* algorithm for robot pathfinding in environments with varying elevation.

## Project Overview

The simulation compares two pathfinding algorithms:
1. **Standard A*** - Uses Euclidean distance as the cost function, finding the shortest path without considering energy consumption.
2. **Battery-Aware A*** - Considers energy consumption due to elevation changes (inclines, declines) in its cost function.

The main hypothesis is that as map size increases, the battery-aware algorithm will achieve longer operating ranges by choosing more energy-efficient paths instead of simply the shortest paths.

## Features

- Random map generation with obstacles, flats, inclines, and declines
- LiFePO4 battery model with realistic energy consumption
- Visualization of paths and comparison metrics
- Performance analysis across different map sizes
- Animation of successful battery-aware paths

## Project Structure

- `map.py` - Map generation and representation
- `battery.py` - LiFePO4 battery model implementation
- `astar.py` - Standard and battery-aware A* algorithm implementations
- `simulation.py` - Simulation engine for running experiments
- `visualization.py` - Tools for visualizing results and creating charts
- `main.py` - Main execution script
- `requirements.txt` - Required Python packages

## Installation

1. Clone this repository
2. Install required packages:
```
pip install -r requirements.txt
```

## Usage

Run the simulation with default parameters:
```
python main.py
```

## Results

The simulation will produce several outputs in the `results` directory:
- Performance comparison charts showing success rates, path distances, and energy consumption
- Visualizations of paths for both algorithms
- Animation of the critical case where the battery-aware algorithm succeeds but the standard algorithm fails

## Battery Model

The LiFePO4 battery model includes:
- Energy consumption calculation based on terrain
- Discharge efficiency
- Regenerative braking on descents

## Customization

You can modify the simulation parameters in `main.py`:
- Map sizes
- Number of trials per size
- Battery capacity
- Obstacle density
- Elevation variance

## License

This project is available for educational and research purposes.
