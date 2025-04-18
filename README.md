# Parallel-Parking-Algorithm-for-Ackermann-Robot

A system for accurate, safe, and convenient automated parking, optimizing parking space utilization.

Full report: [link](https://drive.google.com/drive/folders/1Jxh6SIA5WU8RnSLNCc_X52M6VD_sQgNw?usp=sharing)
## 1. Introduction
Developing autonomous parking systems is necessary for optimizing urban space and enhancing the self driving features of modern vehicles. However, the previous solution suffered from some challenges, such as the inability to smooth trajectory when operated on discontinuous curvature profiles and instability due to inadequate look-ahead distance selection. Consequently, this study proposes an autonomous parking algorithm on the mobile robots model aimed at enhancing accuracy, safety, and driver convenience. To this end,  we introduce an Extended Kalman Filter (EKF) -based localization approach for automatic parallel parking utilizing Inertial measurement unit (IMU)  and Encoder data for path planning and tracking. Additionally, the short-range Lidar is used to enhance environmental perception and enable parking space detection based on point cloud data. Subsequently, we propose the B  Splines algorithm to optimize the parking path adhering to vehicle kinematic constraints while prioritizing smooth, dynamically feasible maneuvers. Last but not least, the Pure Pursuit algorithm combined with a Proportional-Integral controller is presented to ensure reliable tracking with minimized errors.  Building upon a parking space detection accuracy of 98% under laboratory conditions, the proposed controller achieves a peak error of only 1.8 cm with a range of 2.6 cm.  The comprehensive experimental results prove that the proposed algorithm can obtain promising performance against previous methods.
## 2. Methodology

<p align="center">
  <img src="https://github.com/user-attachments/assets/64251342-e07e-4fbd-830e-832397932c56" />
</p>
<p align="center"> Figure 1. Visualization of the automated parallel parking process</p>


<p align="center">
  <img src="https://github.com/user-attachments/assets/f74f8697-9468-4e63-9ec9-145e9fc710d9" width="70%" />
</p>
<p align="center"> Figure 2. Automated Parking System Architecture  </p>

### 2.1 Kinematic Model Construction

<p align="center">
  <img src="https://github.com/user-attachments/assets/795dda57-bc76-4771-8c7e-e1ee96315c64" width=" 50%"/>
</p>
<p align="center"> Figure 3. Vehicle kinematics model  </p>

$$\begin{bmatrix} 
\dot{X}_r \\ 
\dot{Y}_r \\ 
\dot{\phi} 
\end{bmatrix} =
\begin{bmatrix} 
cos(\phi) \\ 
sin(\phi) \\ 
\frac{tan(\delta)}{L} 
\end{bmatrix} v_r
$$

### 2.2 Localization and Parking Space Detection

<p align="center">
  <img src="https://github.com/user-attachments/assets/d7aa5bd5-655d-41e6-b4fe-4a3c81b18dc2" />
</p>
<p align="center"> Figure 4. Sensor Data Fusion for Odometry Using EKF  </p>

The objective is to achieve precise estimation of a mobile robot's complete pose and velocity (6  degrees of freedom) as it navigates its environment. This estimation task can be framed as a nonlinear dynamic system problem:

$$X_k = f(X_{k-1}) + W_{k-1}$$

where  $X_k$ represents the robot's system state (its 3D pose) at time $k$, $f$ is a nonlinear function describing the state transition, and $W_{k-1}$ represents the process noise, assumed to follow a normal distribution. Our state vector $X$, is 3-dimensional, encompassing the vehicle's 3D position, 3D orientation (expressed as Euler angles), and their corresponding velocities. 

$$Z_k = h(X_k) + V_k$$

$$\widehat{X_k} = f(X_{k-1})$$

$$P_k = FP_{k-1}F^T$$

$$K = P_kH^T(HP_kH^T + R)^{-1}$$

$$X_k = \widehat{X_k} + K(Z - H\widehat{X_k})$$

$$P_k = (I - KH)P_k(I - KH)^T + KRK^T$$

Kalman Filter is an optimal filter used to estimate the state of a dynamical system based on noisy measurements. In this case, we use Kalman Filter to estimate the position and orientation of the Ackerman robot car.

State: 

$$x_k = [x, y, \theta]^T$$

represents the position (𝑥,𝑦) and orientation θ of the car at time step k.

Control input 

$$u_k = [v, \delta]^T$$

represents the velocity v and steering angle δ of the car.

Measurement output: 

$$h(x_k) = 
\begin{bmatrix}
\theta_k \\
v_k
\end{bmatrix}$$

represents the measured orientation and velocity from sensors.

The state equation describes how the system state evolves over time, based on the control input:

$$x_{k+1} = f(x_k, u_k) =  
\begin{bmatrix} 
x_k + v_k cos(\theta_k) \Delta t \\ 
y_k + v_k sin(\theta_k) \Delta t \\ 
\theta_k + \frac{v_k}{L} tan(\delta_k) \Delta t
\end{bmatrix}
$$

Jacobian matrix of the state equation with respect to the state:

$$F_k = \frac{\delta f}{\delta x_k} = 
\begin{bmatrix} 
1 & 0 & -v_k sin(\theta_k) \Delta t \\ 
0 & 1 & v_k cos(\theta_k) \Delta t \\ 
0 & 0 & 1
\end{bmatrix}
$$

Jacobian matrix of the state equation with respect to the control input: 

$$P_k = \frac{\delta f}{\delta u_k} = 
\begin{bmatrix} 
cos(\theta_k) \Delta t & 0 \\ 
sin(\theta_k) \Delta t & 0 \\ 
\frac{tan(\delta_k) \Delta t}{L} & \frac{v_k \Delta t}{L cos^2(\delta_k)}
\end{bmatrix}
$$

Jacobian matrix of the measurement equation with respect to the state:

$$H = \frac{\delta h}{\delta x_k} = 
\begin{bmatrix} 
  0 & 0 & 1 \\ 
  0 & 0 & 0 
\end{bmatrix}$$


### 2.3 Path planning for parking manoeuvre and path optimization based B-spline

<p align="center">
  <img src="https://github.com/user-attachments/assets/97d24cea-9883-41af-ac67-35042a065698" />
</p>
<p align="center"> Figure 5. Define coordinates for the B-Spline curve  </p>

The B-Spline curve C(u)  is determined by :

$$C(u) = \sum_{i=0}^{n} N_{i,p}(u)P_i$$

The B-Spline basis function $N_{i,p}(u)$ is determined recursively through the following steps:



* **Zero-degree**
  
$$N_{i,0}(u) = \begin{cases}
0, & u_i < u < u_{i+1} \\
1, & \text{Otherwise}
\end{cases}$$

* **P-th degree**
  
$$N_{i,p}(u) = \frac{u - u_i}{u_{i+p} - u_i} N_{i,p-1}(u) + \frac{u_{i+p+1} - u}{u_{i+p+1} - u_{i+1}} N_{i+1,p-1}(u)$$

### 2.4 Parking path tracking control

<p align="center">
  <img src="https://github.com/user-attachments/assets/587b0efd-4116-498c-873d-1f8d18358981" />
</p>
<p align="center"> Figure 6. Geometric explanation of Pure Pursuit  </p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/5ec4993b-b152-47aa-aa12-df9653088372" />
</p>
<p align="center"> Figure 7. The structure of Pure Pursuit - PI Controller  </p>

Figure 7 illustrates the Ackerman steering principle, which defines the geometric relationship between a vehicle's turning radius and its steering angle. This principle can be mathematically expressed as follows:

$$\frac{sin(2\alpha)}{l_{d}}=\frac{sin(\frac{\pi}{2}-\alpha)}{R}$$

$$\frac{2sin(\alpha)cos(\alpha)}{l_{d}}=\frac{cos(\alpha)}{R}$$

$$\rho=\frac{2sin(\alpha)}{l_{d}}$$

$$\delta_{pp} = tan^{-1}(\frac{L}{R}) = tan^{-1}(\rho L)$$

$$\delta_{pp} = tan^{-1}\left(\frac{2Lsin(\alpha)}{l_{d}}\right)$$

A Proportional-Integral (PI) controller is a feedback control mechanism that utilizes both the proportional (P) and integral (I) components of the error to calculate the control signal. In this context, the error that needs to be controlled is the lateral error, which represents the instantaneous deviation of the vehicle from the desired path .

$$\delta_p = K_p e$$

$$\delta_i = K_i \sum e$$

$$\delta(t) = \delta_{pp} + \delta_p + \delta_i = tan^{-1}\left( \frac{2Lsin(\alpha)}{l_d} \right) + K_p e + K_i \sum e$$




## 3. Simulation

<p align="center">
  <img src="https://github.com/user-attachments/assets/d0e39026-4642-4a30-92ef-e8251bd564d3" width="50%" />
</p>
<p align="center"> Figure 8. Parking Maneuver with Trajectory Visualization on CARLA</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/95c201ec-895b-4176-af7a-c00b0f3dd369" width="50%" />
</p>
<p align="center"> Figure 9. Simulation results in case 1</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/fcbae741-05f8-4419-9d63-bafbae4a4fc3" width="50%" />
</p>
<p align="center"> Figure 10. Simulation results in case 2</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/c832f9d4-e8e2-491c-9f95-d0ca52c1d022" width="50%" />
</p>
<p align="center"> Figure 11. Simulation results in case 3</p>


## 4. Experimental Testing Of Parking Algorithm
<div align="center">
  
**TABLE 1. Vehicle parameters**
| Parameter | Value |
|---|---|
| Wheelbase/m | 0.25 |
| L<sub>f</sub>/m | 0.17 |
| L<sub>r</sub>/m | 0.13 |
| Track/m | 0.22 |
|  δ<sub>max</sub> /rad | 7π/36 | 
| v<sub>max</sub>/ms<sup>-1</sup> | 0.14 |

</div>

<div align="center">

**TABLE 2. Parameters of simulation parking space**

|   | &#124;x<sub>0</sub> - x<sub>1</sub>&#124; (m) | &#124;y<sub>0</sub> - y<sub>1</sub>&#124; (m) | Length (m) | Width (m) |
|:-:|:------------------------------------------:|:------------------------------------------:|:----------:|:---------:|
| 1 | 0.25                                       | 1                                          | 0.9        | 0.25      |
| 2 | 0.3                                        | 1.2                                        | 1          | 0.25      |
| 3 | 0.35                                       | 1.4                                        | 1          | 0.25      |

</div>

<div align="center">
  
**TABLE 3. Actual parking space parameters**

|   | &#124;x<sub>0</sub> - x<sub>1</sub>&#124; (m) | &#124;y<sub>0</sub> - y<sub>1</sub>&#124; (m) | d (m) | Length (m) | Width (m) |
|:-:|:------------------------------------------:|:------------------------------------------:|:-----:|:----------:|:---------:|
| 1 | 0.475                                      | 1.9                                        | 0.65  | 0.9        | 0.25      |
| 2 | 0.425                                      | 1.7                                        | 0.6   | 1          | 0.25      |
| 3 | 0.395                                      | 1.58                                       | 0.57  | 1          | 0.25      |
</div>

<div align="center">
  
**TABLE 4. Location of the vehicle after parking**

| Case  | Front | Behind | Right Side |
|---|---|---|---|
| 1 |  19 cm | 32 cm | 8.5 cm |
| 2 |  18.5 cm | 31 cm | 8.5 cm |
| 3 |  18 cm | 34 cm | 9.5 cm |
</div>

<div align="center">

**TABLE 5. Ablation Study Results**

| Methods                          | Max steering angle<br>step (rad) | Maximum lateral<br>error (cm) |
|:---------------------------------|:-------------------------------:|:----------------------------:|
| Without B-Spline with Fuzzy      | 0.863                           | 4.2                          |
| Without B-Spline with PP-PI      | 0.714                           | 2.1                          |
| B-Spline with Fuzzy              | 0.678                           | 4.1                          |
| B-Spline with Pure Pursuit       | 0.584                           | 3.4                          |
| B-Spline with PP-PI              | 0.023                           | 1.8                          |

</div>

<p align="center">
  <img src="https://github.com/user-attachments/assets/d5e3e598-8097-4631-a32f-60171ea011a0" width="50%" />
</p>
<p align="center"> Figure 12. Block Diagram of the software system structure for a mobile robot</p>


<p align="center">
  <img src="https://github.com/user-attachments/assets/ae225424-696e-40ab-ba63-4a3fb114d565" width="50%" />
</p>
<p align="center"> Figure 13. Hardware architecture of the automated parking system</p>



<p align="center">
  <img src="https://github.com/user-attachments/assets/b9dfd14e-9039-4783-98e4-c36afe3aa2b0" width="50%" />
</p>
<p align="center"> Figure 14. Parking steps in a designed parking lot</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/b18b3634-3251-471c-a7ff-51bff424055b" width="70%" />
</p>
<p align="center"> Figure 15. Experimental results in case 1</p>


<p align="center">
  <img src="https://github.com/user-attachments/assets/120fdbd3-5e2e-4f3f-9899-a20a59a34783" width="70%" />
</p>
<p align="center"> Figure 16. Experimental results in case 2</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/f3994010-1528-432d-99d6-3764c0c41f05" width="70%" />
</p>
<p align="center"> Figure 17. Experimental results in case 3</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/387b43cf-c113-495d-a50d-1cf483d801c7" width="70%" />
</p>
<p align="center"> Figure 18. Fuzzy Membership Functions for CTE, Heading Error, and Steering
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/b889fa84-161e-4857-9705-168462fa89b9" width="70%" />
</p>
<p align="center"> Figure 19. Comparison the tracking results and steering angle between the three controllers Fuzzy, 
Pure-Pursuit and PI-PP
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/150cd4d5-17c6-48fa-b904-1551069eb9d0" width="70%" />
</p>
<p align="center"> Figure 20. Comparison the tracking results and steering angle between the three controllers Fuzzy, 
Pure-Pursuit and PI-PP
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/d5b24b2f-fdfa-489f-bed0-ef99d9ff3f00" width="70%" />
</p>
<p align="center"> Figure 21. Comparison the tracking results and steering angle between the three controllers Fuzzy, 
Pure-Pursuit and PI-PP
</p>






## 5. Conclusion
This study introduces an Automatic Parallel Parking Algorithm for Ackermann-steered robotic vehicles.  The algorithm encompasses three primary tasks: localization, path planning, and trajectory tracking.  Empty parking space detection is achieved by leveraging PointCloud data from an RPlidar sensor in conjunction with geometric algorithms. This method  accurately identifiessuitable parking spots, providing crucial input for the subsequent path planning stage. Unlike traditional Tangent Circle approaches, this research employs a B-Spline trajectory for smoother, more continuous steering commands. This eliminates abrupt transitions between maximum and minimum steering angles, enhancing real-world feasibility.Additionally, the chosen control strategy combines a Pure Pursuit controller with a Proportional Integral (PI) component. In this way, it effectively guides the vehicle along the planned trajectory while mitigating over-reliance on Look-Ahead Distance parameters. However, key limitations include validation conducted solely within controlled laboratory environments, neglecting performance evaluation under adverse weather or complex real-world conditions. Furthermore, the algorithm currently specializes only in parallel parking, indicating potential for expansion to other configurations and necessitating broader environmental robustness testing. Future efforts will concentrate on enhancing perception and localization via camera integration and sensor fusion. Subsequently, path planning and control will be optimized for dynamic environments using adaptive techniques and reinforcement learning, followed by extensive real-world testing for validation and assessing commercialization feasibility.

