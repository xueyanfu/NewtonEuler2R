# NewtonEuler2R

**Newton–Euler Inverse Dynamics for a 2-DOF Robotic Manipulator in Python**

## Overview

NewtonEuler2R is a Python project for studying and implementing the **Recursive Newton–Euler Algorithm (RNEA)** for a two-degree-of-freedom (2-DOF) robotic manipulator.

The project focuses on rigid-body dynamics, coordinate transformations, and the recursive computation of joint torques.

It is intended as an educational implementation for understanding robot dynamics and exploring computational methods used in robotic control.

## Features

The project is designed around the following capabilities:

- Modeling a two-link serial robotic manipulator.
- Computing angular and linear kinematic quantities.
- Implementing forward and backward Newton–Euler recursion.
- Calculating joint torques from joint positions, velocities, and accelerations.
- Supporting further numerical experiments and validation.

## Mathematical Background

The dynamics of a robotic manipulator can be expressed as:

\[
\boldsymbol{\tau}
=
M(\boldsymbol{q})\ddot{\boldsymbol{q}}
+
C(\boldsymbol{q},\dot{\boldsymbol{q}})\dot{\boldsymbol{q}}
+
G(\boldsymbol{q})
\]

where:

| Symbol | Description |
|---|---|
| \(\boldsymbol{q}\) | Joint position vector |
| \(\dot{\boldsymbol{q}}\) | Joint velocity vector |
| \(\ddot{\boldsymbol{q}}\) | Joint acceleration vector |
| \(M(\boldsymbol{q})\) | Joint-space inertia matrix |
| \(C(\boldsymbol{q},\dot{\boldsymbol{q}})\) | Coriolis and centrifugal matrix |
| \(G(\boldsymbol{q})\) | Gravity torque vector |
| \(\boldsymbol{\tau}\) | Joint torque vector |

### Recursive Newton–Euler Algorithm

The algorithm consists of two computational stages.

**1. Forward Recursion**

Computes the angular velocities, angular accelerations, and linear accelerations of successive links, starting from the robot base.

**2. Backward Recursion**

Computes forces, moments, and joint torques, starting from the end-effector and propagating toward the base.

This recursive formulation avoids explicitly constructing the full manipulator inertia matrix when calculating inverse dynamics.

## Installation

Clone the repository:

```bash
git clone https://github.com/xueyanfu/NewtonEuler2R.git
cd NewtonEuler2R
```

Create a Python virtual environment:

```bash
python -m venv .venv
```

Activate the environment on Windows:

```powershell
.venv\Scripts\activate
```

Install the dependencies listed by the project, if a `requirements.txt` file is provided:

```bash
pip install -r requirements.txt
```

## Usage

Open the project in PyCharm or another Python IDE.

Configure the project interpreter to use the virtual environment, locate the main simulation or calculation script, and execute it.

The inverse dynamics calculation conceptually takes:

- Joint positions: `q`
- Joint velocities: `dq`
- Joint accelerations: `ddq`
- Robot link and inertial parameters

and returns the joint torque vector:

```python
tau = [tau1, tau2]
```

The exact execution command and Python API depend on the repository implementation.

## Future Development

Potential extensions include:

- Validation against analytical Lagrangian dynamics.
- Visualization of the 2-DOF manipulator.
- Joint trajectory simulation.
- Comparison with robotics simulation frameworks.
- Extension to manipulators with additional degrees of freedom.

## References

1. John J. Craig, *Introduction to Robotics: Mechanics and Control*.
2. Bruno Siciliano et al., *Robotics: Modelling, Planning and Control*.
3. Kevin M. Lynch and Frank C. Park, *Modern Robotics: Mechanics, Planning, and Control*.

## Author

**Xueyan Fu**

Robotics Engineering  
University of Science and Technology Beijing (USTB)

GitHub: [@xueyanfu](https://github.com/xueyanfu)

## License

No license has been specified yet. See the repository for licensing information.