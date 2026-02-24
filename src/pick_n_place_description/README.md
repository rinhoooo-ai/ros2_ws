# Table Tennis Description

URDF and xacro descriptions for table tennis playing robots.

## Contents

- **urdf/robots/**: Robot definitions with paddle end effector and RealSense camera
- **urdf/end_effectors/paddle/**: Table tennis paddle URDF
- **urdf/sensors/**: RealSense D435 RGBD camera definition

## Usage

This package provides xacro macros for creating Franka FR3 robots equipped with:
- Table tennis paddle end effector
- RealSense D435 RGBD camera mounted on link7

Used by the `table_tennis_gazebo` package for simulation.
