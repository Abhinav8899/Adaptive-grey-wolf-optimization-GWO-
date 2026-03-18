# Augmented Grey Wolf Optimizer (AGWO) for UAV Path Planning

A Python implementation of an **Augmented Grey Wolf Optimizer (AGWO)** specifically tuned for 3D path planning. This project applies bio-inspired optimization to find efficient, collision-free trajectories for autonomous agents like UAVs in complex environments.

## 🛠 Features
* **Bio-Inspired Optimization:** Mimics the leadership hierarchy and hunting mechanism of grey wolves to find optimal paths.
* **3D Obstacle Avoidance:** Integrated collision detection for both box and spherical obstacles with a configurable safety margin.
* **Goal-Biased Search:** Features a high-probability goal bias (80%) to accelerate convergence in large search spaces.
* **Adaptive Maneuvering:** Includes automated "sideways movement" logic to navigate around barriers when the direct path is obstructed.

## 🚀 Getting Started

### Prerequisites
* Python 3.x
* NumPy

```bash
pip install numpy
```
### Installation
1. Clone the repository:
   ```bash
   git clone [https://github.com/Abhinav8899/AGWO.git](https://github.com/Abhinav8899/AGWO.git)
   cd AGWO
   ```

