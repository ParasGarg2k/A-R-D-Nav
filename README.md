# 🧭 A-R-D Nav: A* – RRT – DWA Path Planning System

**A-R-D Nav** is a modular 2D grid-based navigation system that demonstrates path planning using three powerful algorithms: **A\***, **Rapidly-exploring Random Tree (RRT)**, and **Dynamic Window Approach (DWA)**. It supports both **static** and **dynamic** obstacles and visualizes optimal paths in real-time.

---

## 🚀 Features

- 📌 2D Matrix Environment (Grid Map)
- 🧠 Multiple Planning Algorithms:
  - A\* Search (heuristic-based)
  - RRT (sampling-based)
  - DWA (local dynamic obstacle avoidance)
- 🚧 Handles **static** and **moving obstacles**
- 🗺️ Visual representation of map, obstacles, and paths
- 🎥 Video output of simulation
- 🖼️ Snapshot image results

---

## 🧠 Algorithms Overview

| Algorithm | Description | Use Case |
|----------|-------------|----------|
| **A\*** | Graph-based search using heuristics | Best for known static environments |
| **RRT** | Sampling-based path planning | Good for high-dimensional or unknown spaces |
| **DWA** | Local planning with velocity sampling | Real-time dynamic obstacle avoidance |

These algorithms can be run individually or compared in a combined experiment setup.

---

## 📽️ Demo Video & Results

- [Download Demo Video](files/video.mkv)
- 
<video width="600" controls>
  <source src="files/video.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>


  
<p align="center">
  <table>
    <tr>
      <td><img src="files/ip1.png" alt="Intermediate Image" width="400"/></td>
      <td><img src="files/op1.png" alt="Output Result" width="400"/></td>
    </tr>
    <tr>
      <td align="center"><b>Input Image</b></td>
      <td align="center"><b>Output Result</b></td>
    </tr>
  </table>
</p>


---

## 📊 Results

* A\* generates optimal and shortest paths in static grids.
* RRT performs better in complex maps but is non-optimal.
* DWA dynamically avoids moving obstacles with real-time updates.

---

## 📜 License

This project is licensed under the [MIT License](LICENSE).

---

## 🤖 Author

**Paras Garg**
[GitHub](https://github.com/ParasGarg2k) | [LinkedIn](https://www.linkedin.com/in/your-profile)
