# Multi-scale Procrustes-based 3D Shape Control Implementation

This repository contains a basic MATLAB implementation of the concepts presented in the paper titled "Multi-scale Procrustes-based 3D Shape Control" by Ignacio Cuiral-Zueco et al. The paper introduces an innovative method for controlling the shape of 3D objects by employing a multi-scale analysis and a Procrustes-based control strategy. This implementation serves as a starting point for researchers and developers interested in exploring and experimenting with the core ideas presented in the paper.

![Example](example.jpg)

## Introduction

The "Multi-scale Procrustes-based 3D Shape Control" paper addresses the challenge of deforming non-rigid objects into desired shapes through a multi-scale analysis. The method utilizes functional maps to establish correspondences between surface points of the current and target shapes. This correspondence information is then integrated into a Procrustes-based multi-scale control strategy. The primary contribution of the paper is a novel approach that considers a relaxed rigidity assumption, defining scales based on geodesic distances to the grippers, and shaping control actions based on their influence on specific scales.

## Getting Started

### Prerequisites

Before you begin, you should have the following prerequisites:

- MATLAB (R2022b recommended)
- Statistics and Machine Learning Toolbox
- Computer Vision Toolbox
- [iso2mesh toolbox](http://iso2mesh.sf.net)
- Mesh-analysis utils from [ZoomOut implementation](https://github.com/llorz/SGA19_zoomOut/tree/master/utils)

### Usage

To run the implementation, simply open MATLAB and navigate to the project directory. Then, execute the main script `main.m`.

## Contributing

Contributions are welcome! If you would like to contribute, follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Implement your changes and commit with informative messages.
4. Push your changes to your fork.
5. Create a pull request explaining your modifications.

## Citation

If you find this code valuable for your research, please consider citing the original paper:

Ignacio Cuiral-Zueco et al. "Multi-scale Procrustes-based 3D Shape Control." 2023.
