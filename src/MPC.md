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
Given this model of the car, we can now predict outputs for varying sequences of control inputs. Of course, some of these will be better than others. To quantify this notion of "being better", we introduce an objective function (sometimes also referred to as cost function). 	This could, for example, be the eulerian distance to some pre-computed reference point, or the progress along the centre-line. At each time step, the cost depends on both the current state of the car $\mathbf{x_n}$ and the control inputs $\mathbf{u_n}$. We only consider a pre-defined length of control sequences, called the horizon, and add all costs to get an over all cost function. We optimise (minimise the cost, maximise the objective) this function w.r.t the sequence of control inputs.

This optimisation will also be subject to constraints. Some of these are based on physical constraints of the car, such as the steering wheel's maximal steering angle and the motor's maximal speed. Others can be defined by the user; for example we might add constraints on the allowed car states such that a trajectory is only valid if the car stays on the road. As long as the constraints are linear, this does not complicate the calculations carried out by the controller.

From the optimal control sequence calculated by the controller (the one which minimises distance from a reference point or maximises progress along a center line over the length of the control horizon), we only apply the first step (control input) to the car. Then we re-calculate a new optimal sequence with the new state measurements we're getting from the car's sensors.

Because the vehicle model is often quite complicated, both cost function and prediction function may/will need to be linearised to make the problem computationally feasible. This holds especially true since we want the optimisation problem to be convex in order to easily find a global solution and not get stuck in local extrema. This linearisation can be done along the predicted trajectory from the previous time step. Linearisation is only accurate close to the point(s) around which we linearise, but luckily we can assume that the car will drive somewhere near the previously predicted trajectory.

## The vehicle model
# kinematic bicycle model
A simple vehicle model to consider is the kinematic single track model. In this case, no wheel slip is considered (kinematic), both left and right tires are combined together onto a rigid body of mass $m$, the orogin of which is set at the center. The resulting "car" has two wheels, hence it is sometimes also referred to as the kinematic bicycle model.  A schematic of the model can be found in this schematic by [Althoff and Würschling](https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/vehicleModels_commonRoad.pdf): 

![Schematic of kinematic bicycle model](https://github.com/FionaLapp/f1tenth_mpc/blob/master/src/kinematic_bicycle_model_schematic.png) 

Here, the state variables of the car are the position given by 
$\begin{bmatrix}
s_x \\ 
s_y 
\end{bmatrix}$
, the heading angle $\psi$, the velocity $v$ and the steering angle $/delta§. The wheelbase $l_{wb}$ is a constant parameter. Control inputs are the accelleration $a$ and the steering velocity $v_{delta}$.

This leads to the following system of equations:
$$\dot{\delta}=v_{delta}$$
$$\dot{v}=a$$
$$\dot{\psi}=\frac{v}{l_{wb}} * \tan{(\delta)}$$
$$\dot{s_x}=v*\cos{(\psi)}$$
$$\dot{s_y}=v*\sin{(\psi)}$$
The first two equations are true by definition, while the last three are obtained through simple pythagorean geometry.

We can further simplify the system if we assume that both $\delta$ and $v$ are constant over the length of the control horizon. In this case, $v_{\delta}=0$, $a=0$,  and instead we have $\delta$ and $v$ as control inputs while the system becomes:
$$\dot{\psi}=\frac{v}{l_{wb}} * \tan{(\delta)}$$
$$\dot{s_x}=v*\cos{(\psi)}$$
$$\dot{s_y}=v*\sin{(\psi)}$$

# bicycle model
Using the above equations will give reasonably accurate results if the car is driven well below its handling limits. However, as speed increases, tire slip needs to be taken into consideration. This tire slip $\beta$ is defined as the angle of the current velocity with respect to the car's longitudinal axis. It is given by:
$$\beta= \arctan{(\frac{\l_r}{l_{wb}}* \tan{(\delta)})}$$
where $l_r$ denotes the distance from the rear wheel to the centre of mass.
The corresponding state equations from above become:
$$\dot{\psi}=\frac{v}{l_{wb}} * \tan{(\beta)}$$
$$\dot{s_x}=v*\cos{(\psi + \beta)}$$
$$\dot{s_y}=v*\sin{(\psi + \beta)}$$

These are the equations currently used in the code.

## The objective function
Generally, we can differentiate between 1-layer MPC (both path planning and path tracking in one controller) and 2-layer MPC (path planning is done separately from path tracking). The approach used in this implementation is a 2-layer approach that separates path planning and tracking. The MPC controller only concerns itself with tracking a desired path which was pre-computed through an unrelated process.

In this first implementation, the desired path is simply read from a csv file copied from [this repository](https://github.com/f1tenth/f1tenth_racetracks). The race line given in this file minimises the summed curvature while staying within the boundaries of the race track.

Since the file also contains the s-value along the raceline, i.e. the distance travelled at each data point, the controller simply tracks the car's travelled distance and finds the next target point by looking up this distance and selecting the next waypoint.

The objective function is then simply defined as: <code>objective=(target_x - state_x) ** 2 + (target_y - state_y) ** 2</code>

Note that is almost to the Eulerian distance, except that we obstain from taking the square root of the term. Since the square root function is monotonically increasing for positive inputs (and the input consists of squares which are non-negative by definition), we can guarantee that any input which minimises this objective in the target domain would similarly minimise the eulerian distance. Neglecting the root will simply save unnecessary computation.

While this is easy to implement and computationally quick, there are also disadvantages to this approach. Should the car, for whatever reason, deviate from the desired path before coming back to it, the real travelled distance is greater than anticipated and hence the target point will always be ahead of the car, even if the car finds it's way back onto the desired path. In particular, this might lead the car to crash in sharp curves if the target point is far enough ahead, since this implementation has no awareness of road boundaries yet.

