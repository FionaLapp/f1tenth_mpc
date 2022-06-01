# MPC
The code in this repository is used to drive a racecar in the f1tenth racing simulator. 
To run it, the following are required:
- ros, including the following packages:
  - [f1tenth_simulator package](https://github.com/f1tenth/f1tenth_simulator)
  - [hector slam](http://wiki.ros.org/hector_slam)
- [do_mpc](https://www.do-mpc.com/en/latest/)
- scipy, numpy, pandas
Clone the repository into your catkin workspace, build the workspace and run the mpc.launch file.

## Basics
The Controller for the racecar is based on an algorithm called model predictive control (MPC). As the name suggests, MPC uses a model (of the 
laws of physics governing the car's motion) to predict appropriate control inputs. This model can be expressed as a set of differential equations:
$$ \mathbf{\dot{x}}=f(\mathbf{x}, \mathbf{u}) $$
Here, $ \mathbf{x} $ denotes the state of the car, and $ \mathbf{\dot{x}} this state's first time derivative. Its exact components depend on the specifics of the model, but generally, this contains information
about the car's current position, velocity, accelleration, heading angle, etc. Meanwhile, $\mathbf{u}$ denotes the control inputs that are applied to 
the car, e.g. throttle or steering angle.  $f:\mathbb{R}^k \times \mathbb{R}^p \mapsto \mathbb{R}^k$ defines the system of differential equations, where
$k$ is the state space size and $p$ is the control input space size. Intuitively, it mathematically encodes the laws of physics as defined by the model. 

Since a computer can only calculate new control inputs in finitely small time steps, it is necessary to discretise this system of differential equations 
by turning it into a system of difference equations:
$$ \mathbf{x}(t+T)=f(\mathbf{x}(t), \mathbf{u}(t))$$
where both $\mathbf{x}$ and $\mathbf{u}$ are functions of time, and $T$ is a selected time step. Without loss of generality, we can set $T=1$, and denote $\mathbf{x}(t)= \mathbf{x_n}$ to get:
$$ \mathbf{x_{n+1}}=f(\mathbf{x_n}, \mathbf{u_n})$$
