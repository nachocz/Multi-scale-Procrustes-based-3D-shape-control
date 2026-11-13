# Multi-scale Procrustes-based 3D shape control

The **[interactive demos](https://ignaciocuiral.com/Multi-scale-Procrustes-based-3D-shape-control/)** explain the method step by step and compute it in the browser. The web version of the paper, with its figures and references, is at **[ignaciocuiral.com/Multi-scale-Procrustes-based-3D-shape-control/paper/](https://ignaciocuiral.com/Multi-scale-Procrustes-based-3D-shape-control/paper/)**.

## Interactive demos

**[Open the demos](https://ignaciocuiral.com/Multi-scale-Procrustes-based-3D-shape-control/)**: the method computed in the browser, on objects simulated with ARAP as in the simulations of the paper.

| Page | Content |
|---|---|
| `docs/index.html` | Overview, the video of the paper, its figures, and the code. |
| `docs/intro.html` | **The idea in 2D**: flat pieces moved in the plane (SE(2)). The geodesic ball of radius r, the local-rigidity model at r against the simulated object for a motion of a gripper, the estimation of r from the observed responses to small motions, and the closed loop. |
| `docs/analysis.html` | **Multi-scale analysis** (Secs. II-A to II-D) at one instant of the 3D closed loop: Procrustes transforms of the geodesic balls, the estimate for an action scale and an influence scale, the error increments, their validation w(t, r'), and q(t, r). |
| `docs/control.html` | **Shape control** (Secs. II-E and IV-A): the closed loop on five 3D objects, with the isolines of r*, the poses that the Procrustes transforms prescribe, the model estimate against the simulated response, and the plots of Fig. 2. |
| `docs/stability.html` | **Stability** (Sec. III): the terms of the lemmas and the Lyapunov function along the closed loop. |
| `docs/paper/` | The web version of the paper. |

The pages are static files (KaTeX and three.js included). The simulation modules (`docs/js/sim`) also run in Node.js.

```bash
python run.py                  # serves docs/ and opens http://127.0.0.1:8765/
python tools/build_web.py      # after editing the sources in docs/ (needs Node.js; npx fetches esbuild)
node tests/test_sim.mjs        # checks of the simulation modules (about 40 s)
```

KaTeX and three.js keep their MIT licenses (`docs/vendor`), and the optional background music keeps the licenses listed in `docs/audio/CREDITS.md`.

The demos were developed with the assistance of Claude (Anthropic), an AI model, which wrote and tested the code and drafted the text of the pages under the direction of the authors.

---

## Citation

If you find this work useful, please cite the following publication:

> **[1]** I. Cuiral-Zueco and G. López-Nicolás, "Multiscale Procrustes-Based 3-D Shape Control", in *IEEE/ASME Transactions on Mechatronics*, vol. 29, no. 3, pp. 1738-1748, 2024, [doi: 10.1109/TMECH.2023.3325934](https://doi.org/10.1109/TMECH.2023.3325934).

```bibtex
@ARTICLE{10311090,
  author = {Cuiral-Zueco, Ignacio and López-Nicolás, Gonzalo},
  title = {Multiscale Procrustes-Based 3-D Shape Control},
  journal = {IEEE/ASME Transactions on Mechatronics},
  year = {2024},
  volume = {29},
  number = {3},
  pages = {1738-1748},
  doi = {10.1109/TMECH.2023.3325934},
  keywords = {Shape control;Grippers;Deformation;Task analysis;Rigidity;Jacobian matrices;Object detection;Deformable models;Multi-robot systems;Deformable object manipulation;multirobot manipulation;shape control;shape servoing}
}
```

## Authors

- [Ignacio Cuiral-Zueco](https://nachocz.github.io/ignacio-cuiral-zueco.github.io/)
- [Gonzalo López-Nicolás](http://webdiis.unizar.es/~glopez)


# Implementation

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

# Acknowledgements

This work was supported through Project REMAIN S1/1.1/E0111 (Interreg Sudoe Programme, ERDF), Project PID2021-124137OB-I00, and Project TED2021-130224B-I00 funded in part by MCIN/AEI/10.13039/501100011033, in part by the ERDF A way of making Europe, and in part by the European Union NextGenerationEU/PRTR.

<img src="https://github.com/user-attachments/assets/44c6e2a3-e281-4e59-a594-fcf72769a4b9" height="100px">
<img src="https://github.com/user-attachments/assets/ed25931c-3dc5-4db3-8c63-13883d3ef100" height="100px">
