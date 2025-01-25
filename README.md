# Boids Simulation

<img src="https://github.com/user-attachments/assets/0aefaf92-051a-4104-964a-c1cfcf4b589c"/>

This is an implementation of the Boids algorithm (described [here](https://www.red3d.com/cwr/boids/)), created as a project for the class Graphic Processors in Computational Applications at Warsaw University of Technology. 

## Overview
The program features two modes of operation:
- GPU, which uses CUDA for acceleration and CUDA-OpenGL interoperability layer for rendering
- CPU, which uses standard C++ and OpenGL
You may switch freely between the modes and adjust simulation parameters directly in the UI.

## Building and Running

### Prerequisites
Make sure you have the following installed on your system:
- **CUDA Toolkit**: Install from [NVIDIA's website](https://developer.nvidia.com/cuda-toolkit).
- **OpenGL**: Usually pre-installed on most systems. If not, install it via your system's package manager.
- **CMake**: Install from [the official website](https://cmake.org/).

### Steps to Build and Run

1. **Clone the Repository**:
   ```
   git clone https://github.com/your-username/boids-sim.git
   cd boids-sim
   ```
2. Create a Build directory
   ```
   mkdir build
   cd build
   ```
3. Configure the project with CMake
   ```
   cmake ..
   ```
4. Build the project
   ```
   cmake --build .
   ```
