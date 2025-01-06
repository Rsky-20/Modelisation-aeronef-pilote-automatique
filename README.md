# Au511 - Modelisation Aéronef Pilote Automatique

|                       | **Details**                                            |
|-----------------------|--------------------------------------------------------|
| **Date of Creation**  | 13.11.2024                                             |
| **Author**            | Valentin DESFORGES - Pierre VAUDRY                     |
| **Version**           | v6.01.2024                                             |
| **Python Version**    | Python 3.12.X                                          |
| **Key Libraries Used**| `control`, `scipy`, `matplotlib`, `numpy`, `inquirer`  |

---

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. [General Methodology](#general-methodology)
4. [Installation Instructions](#installation-instructions)
5. [How to Run the Code](#how-to-run-the-code)
6. [Results](#results)
7. [Future Improvements](#future-improvements)
8. [References](#references)

---

## Introduction

This project is part of the **Control of Aircraft** coursework and focuses on the modeling and control of a fighter aircraft (MIRAGE III class). The goal is to understand the dynamic behavior of an aircraft, analyze its stability, and develop control loops to enhance its performance. The project uses Python's scientific computing ecosystem to implement and analyze control algorithms.

---

## Features

- **Aircraft Modeling**: Comprehensive modeling of an aircraft's dynamics, including aerodynamic coefficients and stability analysis.
- **Interactive Menu**: A user-friendly CLI to navigate through the steps of analysis and control design.
- **State Space Representation**: Calculation and display of state-space matrices and transfer functions.
- **Feedback Control Design**: Implementation of feedback loops for stability and performance improvement.
- **Dynamic Simulation**: Analysis of open-loop and closed-loop responses using tools like `SISOPY31`.

---

## General Methodology

### **Aircraft Characteristics**
- Detailed modeling of the MIRAGE III class aircraft, including aerodynamic and mass properties.
- Utilization of the US 76 Standard Atmosphere Model for atmospheric calculations.

### **Control Design**
- Calculation of equilibrium points and state-space matrices.
- Analysis of open-loop modes and transient response.
- Design of feedback loops (e.g., pitch rate, angle of attack).

### **Tools Used**
- `SISOPY31`: A Python-based tool equivalent to MATLAB's SISO Tool for root locus and control design.

---

## Installation Instructions

1. **Install Python 3.11 or later.**
   - Ensure `pip` is installed.

2. **Clone the repository and navigate to the project directory:**
   ```bash
   git clone https://github.com/your-repo-url/control-of-aircraft
   cd control-of-aircraft
   ```

3. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Ensure additional Python packages are installed (if not already):**
   ```bash
   conda install -c conda-forge slycot control
   ```

5. **Prepare the atmospheric data file:**
   Place `ussa76ut86.txt` in the `./data/` directory.

---

## How to Run the Code

1. Launch the main script:
   ```bash
   python main.py
   ```

2. Follow the interactive menu to select:
   - Aircraft type
   - Operations (e.g., equilibrium point analysis, transient phase study, feedback loop design).

3. Outputs, including equilibrium conditions, state-space matrices, and plots, are displayed or saved in the `data/fig/` directory.

---

## Results

### Key Outputs:
- **Equilibrium Conditions**: Computed coefficients, angles, and forces at equilibrium.
- **State-Space Representation**: Matrices (A, B, C, D) for dynamic modeling.
- **Transient Response**: Step responses of open and closed-loop systems.
- **Root Locus**: Interactive tool to adjust control gains and analyze pole locations.

### Example Plot:
*(Include a representative plot or figure here.)*

---

## Future Improvements

- Extend modeling to include lateral dynamics.
- Enhance the GUI for more intuitive interaction.
- Integrate advanced control methods like LQR or MPC.

---

## References

1. [Practical Work](.\docs\PracticalWork_ControlOfAricraft2024.pdf)
2. [Python Control](.\docs\pythoncontrol20232024.pdf)
