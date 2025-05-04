# Robot Path Visualization Tool

A Python-based 3D visualization tool for robot path planning and quaternion visualization. Built with PyQt6 and PyQtGraph.

## Features

- 3D visualization of robot paths and waypoints
- Obstacle visualization and collision detection
- Quaternion visualization and conversion
- Path import/export in CSV format
- Interactive 3D view with coordinate axes and grid

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd <repository-name>
```

2. Create and activate a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage

Run the application:
```bash
python main.py
```

### Path Planning Tab
- Add, remove, and edit path points
- Visualize path in 3D space
- Check for collisions with obstacles
- Import/export paths in CSV format
- Use Quaternion Visualizer Tab to easily calculate desired quaternion rotations

### Quaternion Visualizer Tab
- Input roll, pitch, and yaw angles
- Visualize orientation in 3D
- Convert between Euler angles and quaternions
- Copy quaternion values to clipboard

## File Format

### Path CSV Format
The path file should be a CSV with the following columns:
- Name: Point identifier
- X, Y, Z: Position coordinates
- qW, qX, qY, qZ: Quaternion components

## Dependencies

- Python 3.x
- PyQt6
- PyQtGraph
- NumPy
