# Battery State Aware A* Simulation

This project demonstrates the performance difference between a battery state aware A* algorithm and a standard A* algorithm for robot pathfinding in environments with varying elevation.

## Project Overview

The simulation compares two pathfinding algorithms:
1. **Standard A*** - Uses Euclidean distance as the cost function, finding the shortest path without considering energy consumption.
2. **Battery-Aware A*** - Considers energy consumption due to elevation changes (inclines, declines) and terrain in its cost function.

The main hypothesis is that as map size increases, the battery-aware algorithm will achieve longer operating ranges by choosing more energy-efficient paths rather than simply the shortest paths. This is particularly relevant for robots operating in environments with significant elevation changes, where battery management is crucial for extended operation.

## Features and Functionality

- **City-like Map Generation**: Creates environments with roads, buildings, and varying elevations including hills and valleys
- **Physics-based Battery Model**: LiFePO4 battery with state-of-charge dependent efficiency and regenerative braking
- **Dual Pathfinding Algorithms**:
  - Standard A*: Optimizes for shortest distance
  - Battery-aware A*: Optimizes for energy efficiency considering terrain and elevation
- **Comprehensive Analysis**: Path characteristics, energy usage, and terrain choice comparisons
- **Visualizations**: Path comparisons, energy profiles, and critical case identification

## Project Structure

- `map.py` - City-style map generation with elevation features
- `battery.py` - Physics-based LiFePO4 battery model with regenerative braking
- `astar.py` - Standard and battery-aware A* algorithm implementations
- `simulation.py` - Simulation engine and path analysis tools
- `visualization.py` - Result visualization functions
- `main.py` - Simulation execution and performance reporting
- `requirements.txt` - Required Python packages

## Installation

### Option 1: Standard Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/soc-astar.git
   cd soc-astar
   ```

2. Install required packages:
   ```bash
   pip install -r requirements.txt
   ```

### Option 2: Using Virtual Environment (Recommended)

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/soc-astar.git
   cd soc-astar
   ```

2. Create a virtual environment:
   ```bash
   # Using venv (Python 3.3+)
   python -m venv venv
   
   # Activate the virtual environment
   # On Windows
   venv\Scripts\activate
   
   # On macOS/Linux
   source venv/bin/activate
   ```

3. Install required packages in the virtual environment:
   ```bash
   pip install -r requirements.txt
   ```

4. When you're done, you can deactivate the virtual environment:
   ```bash
   deactivate
   ```

### Requirements

The major dependencies include:
- Python 3.6+
- NumPy: For numerical operations and array handling
- Matplotlib: For visualization and chart generation
- Other dependencies listed in requirements.txt

## Usage

Run the simulation with:
```bash
python main.py
```

You can customize the simulation in `main.py`:
- Map sizes array (smaller sizes for quick testing, larger for more prominent differences)
- Number of trials per map size
- Battery capacity (adjust to make energy constraints more or less stringent)

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
