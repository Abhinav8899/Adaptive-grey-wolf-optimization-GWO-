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
   git clone https://github.com/Abhinav8899/Adaptive-grey-wolf-optimization-GWO-.git
   cd Adaptive-grey-wolf-optimization-GWO-
   ```

## 📊 Performance Benchmarking

The AGWO implementation is evaluated based on its ability to minimize path length while maintaining a 100% collision-free trajectory. The provided benchmark scripts simulate different 3D obstacle densities to test the optimizer's convergence.

### How to Run
Execute the following scripts to see the solver in action:

```bash
# Scenario 1: Open space with sparse obstacles
python benchmarksingle1.py

# Scenario 2: Intermediate complexity (Mixed box and sphere clusters)
python benchmarksingle2.py

# Scenario 3: High-density 3D maze environment
python benchmarksingle3.py
```

## 🤝 Contributing
Feel free to fork this repository and submit pull requests. For major changes, please open an issue first to discuss what you would like to change.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
