# CrackScanSim

## Overview

CrackScanSim is a ROS 2 prototype for a mobile rover that scans large metallic surfaces using ultrasonic transducers. It simulates how a robot can traverse a defined area, perform non-destructive testing (NDT), and detect flaws such as cracks and corrosion.

This system is intended as a stepping stone toward real-world deployment with actual robot hardware and sensor integration.

## Components

The system consists of four ROS 2 nodes:

- `rover_position_node`: Simulates rover movement in a raster pattern over the plate.
- `sensor_position_node`: Calculates the positions where the ultrasonic sensor performs readings based on the rover's pose.
- `sensor_value_node`: Emits simulated sensor readings based on predefined probabilities.
- `matrix_visualization_node`: Displays a live visualization of the scanned surface and updates in real time.

All components are modular and designed for easy substitution with real-world data sources or drivers.

## Configuration

The scanning parameters can be adjusted in `config/plate_config.yaml`:

```yaml
plate:
  width: 50              # Width of the metal plate (cm)
  height: 30             # Height of the metal plate (cm)
  rover_step_x: 30       # Rover step size in X (cm)
  rover_step_y: 1        # Rover step size in Y (cm)
  sensor_width: 30       # Sweep width of the sensor (cm)
  sensor_step: 1         # Step resolution within a sweep (cm)
```

## Extending to Real Hardware

This prototype can be extended by:

- Replacing `rover_position_node` with real odometry or tf data from a mobile robot.
- Substituting `sensor_value_node` with real drivers for ultrasonic, EMAT, or infrared sensors.
- Integrating with `ros2_control` for motor actuation and feedback.
- Using transform lookups (`tf2`) to compute real sensor positions from the robot's base frame.


