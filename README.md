# Modular Drone Control System

A modular, local-first Python architecture for controlling drones (initially DJI Tello).

## Architecture

The system is built on three core interfaces:
1.  **IDrone**: Abstracts the physical hardware (e.g., Tello).
2.  **IVision**: Abstracts the computer vision logic (e.g., Aruco, MediaPipe).
3.  **IController**: Abstracts the flight logic (e.g., Follow, Patrol).

## Prerequisites

*   `uv` (latest)

## Usage

1.  Connect to Tello WiFi.
2.  Run the main script using `uv`:
    ```bash
    uv run main.py
    ```

## Structure

*   `interfaces/`: Abstract Base Classes.
*   `hardware/`: Drone drivers.
*   `vision/`: Computer vision modules.
*   `control/`: Flight control logic.
