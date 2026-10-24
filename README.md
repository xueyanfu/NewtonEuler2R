# NewtonEuler2R

**Recursive Newton–Euler Inverse Dynamics for a 2-DOF Robotic Manipulator**

A Python-based educational project exploring the inverse dynamics of a two-link robotic manipulator using the Recursive Newton–Euler Algorithm (RNEA).

**Language:** Python  
**Domain:** Robotics · Robot Dynamics · Rigid-Body Mechanics  
**Author:** Xueyan Fu

---

## 1. Overview

NewtonEuler2R explores the mathematical modeling and numerical computation of rigid-body dynamics for a two-degree-of-freedom (2-DOF) serial robotic manipulator.

The project focuses on the Recursive Newton–Euler Algorithm (RNEA), a computationally efficient method for determining the joint torques required to achieve specified joint motions.

The main objectives are to:

- Understand the relationship between joint motion and actuator torque.
- Study angular and linear motion propagation along a kinematic chain.
- Apply Newton's laws and Euler's rotational equations to robotic systems.
- Explore recursive algorithms for robot inverse dynamics.

## 2. Robot Model

The system considered is a serial manipulator with two revolute joints.

Its generalized coordinates are:

- `q = [q1, q2]`: joint angles, in radians.
- `dq = [dq1, dq2]`: joint angular velocities, in rad/s.
- `ddq = [ddq1, ddq2]`: joint angular accelerations, in rad/s².

The required actuator torques are represented as:

`tau = [tau1, tau2]`

The rigid-body model may involve link masses, link lengths, center-of-mass positions, inertia tensors, and gravitational acceleration.

## 3. Mathematical Foundation

The standard inverse dynamics equation is:

$$
\boldsymbol{\tau}
=
M(\boldsymbol{q})\ddot{\boldsymbol{q}}
+
C(\boldsymbol{q},\dot{\boldsymbol{q}})\dot{\boldsymbol{q}}
+
G(\boldsymbol{q})
$$

where:

| Symbol | Description |
|---|---|
| q | Joint position vector |
| dq | Joint velocity vector |
| ddq | Joint acceleration vector |
| M(q) | Joint-space inertia matrix |
| C(q, dq) | Coriolis and centrifugal matrix |
| G(q) | Gravity torque vector |
| tau | Joint torque vector |

The Recursive Newton–Euler Algorithm evaluates inverse dynamics through recursive rigid-body calculations, without explicitly assembling the full joint-space inertia matrix.

### 3.1 Forward Recursion

The forward pass propagates kinematic information from the base to the end-effector.

For each link, the relevant quantities include:

- Angular velocity and angular acceleration.
- Linear acceleration of the link coordinate frame.
- Linear acceleration of the link's center of mass.
- Inertial forces and moments.

The calculations account for relative joint motion and transformations between adjacent coordinate frames.

### 3.2 Backward Recursion

The backward pass propagates forces and moments from the terminal link toward the base.

Each link contributes inertial forces and moments, while the transmitted wrench accounts for contributions from downstream links.

The joint torque is obtained by projecting the resulting moment onto the corresponding revolute joint axis.

### 3.3 Computational Workflow

```text
Joint States + Robot Parameters
                |
                v
     Coordinate Transforms
                |
                v
       Forward Recursion
                |
                v
    Link Accelerations
                |
                v
   Inertial Forces & Moments
                |
                v
       Backward Recursion
                |
                v
       Joint Torques
         [tau1, tau2]
```

For a serial manipulator with n joints, RNEA has O(n) computational complexity.

## 4. Getting Started

### Clone the Repository

```bash
git clone https://github.com/xueyanfu/NewtonEuler2R.git
cd NewtonEuler2R
```

### Create a Virtual Environment

```bash
python -m venv .venv
```

Activate it on Windows PowerShell:

```powershell
.\.venv\Scripts\Activate.ps1
```

On Linux or macOS:

```bash
source .venv/bin/activate
```

### Install Dependencies

If the repository includes a `requirements.txt` file:

```bash
python -m pip install -r requirements.txt
```

Otherwise, install the packages imported by the source files.

### Run the Project

Open the repository in PyCharm or another Python IDE.

Select the virtual environment as the project interpreter and run the appropriate project entry-point script.

**Note:** The exact entry-point filename and command need to be verified against the current source tree.

## 5. Inputs and Outputs

The inverse dynamics calculation maps the robot's kinematic state and physical parameters to the required joint torques.

| Input | Meaning | Unit |
|---|---|---|
| `q` | Joint angles | rad |
| `dq` | Joint velocities | rad/s |
| `ddq` | Joint accelerations | rad/s² |
| Link mass | Mass of each link | kg |
| Center of mass | Position relative to link frame | m |
| Inertia tensor | Rotational inertia | kg·m² |
| Gravity | Gravitational acceleration | m/s² |

**Output:** A two-element joint torque vector, measured in N·m.

```text
Inverse Dynamics

Inputs:
  q    = [q1, q2]
  dq   = [dq1, dq2]
  ddq  = [ddq1, ddq2]

Output:
  tau  = [tau1, tau2]
```

This represents the computational interface conceptually; the actual Python API depends on the implementation.

## 6. Validation

A useful validation strategy is to compare the recursive Newton–Euler result against an independently derived analytical dynamics model.

Recommended checks include:

- Static configurations with zero joint velocities and accelerations.
- Motion with gravity disabled.
- Nonzero joint accelerations to evaluate inertial effects.
- Numerical comparison with a Lagrangian dynamics formulation.

These are proposed validation cases; no numerical accuracy or test coverage is claimed here.

## 7. Future Work

Potential improvements include:

- Analytical validation against Lagrangian dynamics.
- Automated unit tests for representative joint states.
- Visualization of the two-link manipulator.
- Joint-space trajectory simulation and torque plotting.
- Extension to higher-degree-of-freedom robotic manipulators.

## 8. References

1. Craig, J. J. *Introduction to Robotics: Mechanics and Control*.
2. Lynch, K. M., and Park, F. C. *Modern Robotics: Mechanics, Planning, and Control*.
3. Featherstone, R. *Rigid Body Dynamics Algorithms*.

## 9. Author

**Xueyan Fu**  
Robotics Engineering Undergraduate  
University of Science and Technology Beijing (USTB)

GitHub: [xueyanfu](https://github.com/xueyanfu)

## 10. License

No specific open-source license is declared in this README. Refer to the repository's `LICENSE` file if one is added.

---

*Developed for robotics dynamics study and algorithm implementation.*