# Multi-scale Procrustes-based 3D Shape Control Implementation

This repository contains a basic MATLAB implementation of the concepts presented in the paper

> **[1]** I. Cuiral-Zueco and G. López-Nicolás, "Multiscale Procrustes-Based 3-D Shape Control," in *IEEE/ASME Transactions on Mechatronics*, vol. 29, no. 3, pp. 1738-1748, June 2024, [doi: 10.1109/TMECH.2023.3325934](https://doi.org/10.1109/TMECH.2023.3325934).

The paper introduces an innovative method for controlling the shape of 3D objects by employing a multi-scale analysis and a Procrustes-based control strategy. This implementation serves as a starting point for researchers and developers interested in exploring and experimenting with the core ideas presented in the paper.

![example](https://github.com/nachocz/Multi-scale-Procrustes-based-3D-shape-control/assets/29798564/4e92d5d7-3ce0-48bd-b74d-878f35c3b298)

## Introduction

The "Multi-scale Procrustes-based 3D Shape Control" paper addresses the challenge of deforming non-rigid objects into desired shapes through a multi-scale analysis. The method utilizes functional maps to establish correspondences between surface points of the current and target shapes. This correspondence information is then integrated into a Procrustes-based multi-scale control strategy. The primary contribution of the paper is a novel approach that considers a relaxed rigidity assumption, defining scales based on geodesic distances to the grippers, and shaping control actions based on their influence on specific scales.

## Getting Started


### Prerequisites

Before you begin, you should have the following prerequisites:

- MATLAB (R2022b recommended)
- Statistics and Machine Learning Toolbox
- Computer Vision Toolbox
- [iso2mesh toolbox](http://iso2mesh.sf.net)
- [gptoolbox](https://github.com/alecjacobson/gptoolbox)

The utils in "dependencies.zip" are Mesh-analysis utils from [ZoomOut implementation](https://github.com/llorz/SGA19_zoomOut/tree/master/utils) to which we performed slight modifications, thus including them as a ".zip".

### Usage

To run the implementation, simply open MATLAB and navigate to the project directory. Then, execute the main script `main.m`.

## Contributing

Contributions are welcome! If you would like to contribute, follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Implement your changes and commit with informative messages.
4. Push your changes to your fork.
5. Create a pull request explaining your modifications.

---
# Citation

The related paper can be cited as:

> **[1]** I. Cuiral-Zueco and G. López-Nicolás, "Multiscale Procrustes-Based 3-D Shape Control," in *IEEE/ASME Transactions on Mechatronics*, vol. 29, no. 3, pp. 1738-1748, June 2024, [doi: 10.1109/TMECH.2023.3325934](https://doi.org/10.1109/TMECH.2023.3325934).

- [Ignacio Cuiral-Zueco](https://nachocz.github.io/icz-cv/)
- [Gonzalo López-Nicolás](http://webdiis.unizar.es/~glopez)

# Acknowledgements

This work was supported through Project REMAIN S1/1.1/E0111 (Interreg Sudoe Programme, ERDF), Project PID2021-124137OB-I00, and Project TED2021-130224B-I00 funded in part by MCIN/AEI/10.13039/501100011033, in part by the ERDF A way of making Europe, and in part by the European Union NextGenerationEU/PRTR.

<img src="https://github.com/user-attachments/assets/44c6e2a3-e281-4e59-a594-fcf72769a4b9" height="100px">
<img src="https://github.com/user-attachments/assets/ed25931c-3dc5-4db3-8c63-13883d3ef100" height="100px">
