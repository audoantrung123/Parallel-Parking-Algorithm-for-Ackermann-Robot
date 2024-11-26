# Parallel-Parking-Algorithm-for-Ackermann-Robot

An Off-policy Reinforcement Learning Algorithm for Optimal Tracking Control Problem

Full report: [link](https://drive.google.com/drive/folders/1Jxh6SIA5WU8RnSLNCc_X52M6VD_sQgNw?usp=sharing)
## 1. Introduction
Recognizing the increasing prevalence of self parking features in modern vehicles, aimed at enhancing accuracy, safety, and driver convenience, this study explores autonomous parking control on mobile robots. This study explores autonomous parking control on mobile robots, offering three key contributions. Firstly, it delivers an efficient platform for research and education, enabling researchers and students to readily grasp relevant concepts, algorithms, and techniques without requiring investment in expensive systems. Furthermore, the study prioritizes safety by utilizing mobile robots for algorithm testing, minimizing collision risks and potential damage. Finally, the robot software system, developed using the Robot Operating System (ROS) and a distributed architecture, ensures high flexibility, scalability, and reusability, facilitating seamless integration and sharing across diverse robotic projects. This study specifically proposes an EKF-based localization approach for automatic parallel parking, utilizing IMU and Encoder data for path planning and tracking. Short-range lidar enhances environmental perception, enabling parking space detection based on point cloud data. Algorithm B Splines algorithm optimizes the parking path, adhering to vehicle kinematic constraints while prioritizing smooth, dynamically feasible maneuvers. The chosen Pure Pursuit algorithm combined with Proportional-Integral controller, valued for its simplicity and stability, ensures reliable tracking with minimized errors.
## 2. Methodology

<p align="center">
  <img src="https://github.com/user-attachments/assets/64251342-e07e-4fbd-830e-832397932c56" />
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/f85e3cd0-5547-4d8c-a7a8-4e9eccae21a3" />
</p>


### 2.1 Kinematic Model Construction

<p align="center">
  <img src="https://github.com/user-attachments/assets/795dda57-bc76-4771-8c7e-e1ee96315c64" />
</p>

### 2.2 Localization and Parking Space Detection

<p align="center">
  <img src="https://github.com/user-attachments/assets/d7aa5bd5-655d-41e6-b4fe-4a3c81b18dc2" />
</p>

$$\begin{bmatrix} \dot{X_r} \\ \dot{Y_r} \\ \dot{\varphi}  \end{bmatrix} = \begin{bmatrix} cos(\varphi) & 0 \\ sin(\varphi) & 0 \\ \frac{tan(\delta)}{L} & 0 \end{bmatrix} v_r$$

$$X_k = f(X_{k-1}) + W_{k-1}$$

$$Z_k = h(X_k) + V_k$$

$$\widehat{X_k} = f(X_{k-1})$$

$$P_k = FP_{k-1}F^T$$

$$K = P_kH^T(HP_kH^T + R)^{-1}$$

$$X_k = \widehat{X_k} + K(Z - H\widehat{X_k})$$

$$P_k = (I - KH)P_k(I - KH)^T + KRK^T$$

$$x_k = [x, y, \theta]^T$$

$$u_k = [v, \delta]^T$$

$$h(x_k) = \begin{bmatrix} \theta_k \\ v_k \end{bmatrix}$$

$$x_{k+1} = f(x_k, u_k) = \begin{bmatrix} x_k + v_k cos(\theta_k) \Delta t \\ y_k + v_k sin(\theta_k) \Delta t \\ \theta_k + \frac{v_k}{L} tan(\delta_k) \Delta t \end{bmatrix}$$

$$F_k = \frac{\delta f}{\delta x_k} = \begin{bmatrix} 1 & 0 & -v_k sin(\theta_k) \Delta t \\ 0 & 1 & v_k cos(\theta_k) \Delta t \\ 0 & 0 & 1 \end{bmatrix}$$

$$P_k = \frac{\delta f}{\delta u_k} = \begin{bmatrix} cos(\theta_k) \Delta t & 0 \\ sin(\theta_k) \Delta t & 0 \\ \frac{tan(\delta_k) \Delta t}{L} & \frac{v_k \Delta t}{L cos^2(\delta_k)} \end{bmatrix}$$

$$H = \frac{\delta h}{\delta x_k} = \begin{bmatrix} 0 & 0 & 1 \\ 0 & 0 & 0 \end{bmatrix}$$



### 2.3 Path planning for parking manoeuvre and path optimization based B-spline

<p align="center">
  <img src="https://github.com/user-attachments/assets/97d24cea-9883-41af-ac67-35042a065698" />
</p>

$$N_{i,p}(u) = \frac{u - u_i}{u_{i+p} - u_i} N_{i,p-1}(u) + \frac{u_{i+p+1} - u}{u_{i+p+1} - u_{i+1}} N_{i+1,p-1}(u)$$

$$C(u) = \sum_{i=0}^{n} N_{i,p}(u)P_i$$

$$N_{i,0}(u) = \begin{cases}
0, & u_i < u < u_{i+1} \\
1, & \text{Otherwise}
\end{cases}$$


### 2.4 Parking path tracking control

<p align="center">
  <img src="https://github.com/user-attachments/assets/587b0efd-4116-498c-873d-1f8d18358981" />
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/5ec4993b-b152-47aa-aa12-df9653088372" />
</p>

$$\frac{sin(2\alpha)}{l_{d}}=\frac{sin(\frac{\pi}{2}-\alpha)}{R}$$

$$\frac{2sin(\alpha)cos(\alpha)}{l_{d}}=\frac{cos(\alpha)}{R}$$

$$\rho=\frac{2sin(\alpha)}{l_{d}}$$

$$\delta_{pp} = tan^{-1}(\frac{L}{R}) = tan^{-1}(\rho L)$$

$$\delta_{pp} = tan^{-1}\left(\frac{2Lsin(\alpha)}{l_{d}}\right)$$

$$\delta_p = K_p e$$

$$\delta_i = K_i \sum e$$

$$\delta(t) = \delta_{pp} + \delta_p + \delta_i = tan^{-1}\left( \frac{2Lsin(\alpha)}{l_d} \right) + K_p e + K_i \sum e$$




## 3. Simulation

## 4. Experrimental Testing Of Parking Algorithm

## 5. Conclusion
This study introduces an Automatic Parallel Parking Algorithm for Ackermann-steered robotic vehicles.  The algorithm encompasses three primary tasks: localization, path planning, and trajectory tracking.  Empty parking space detection is achieved by leveraging PointCloud data from an RPlidar sensor in conjunction with geometric algorithms. This method demonstrates accuracy in identifying suitable parking spots, providing crucial input for the subsequent path planning stage.
Departing from traditional Tangent Circle approaches, this research employs a B-Spline trajectory for smoother, more continuous steering commands. This eliminates abrupt transitions between maximum and minimum steering angles, enhancing real-world feasibility.  The chosen control strategy combines a Pure Pursuit controller with a Proportional Integral (PI) component. This combination proves effective in guiding the vehicle along the planned trajectory while mitigating over-reliance on Look-Ahead Distance parameters.
Future development will prioritize enhancing detection and localization capabilities by integrating a camera for lane detection, parking sign recognition, and vehicle identification. This visual data, combined with existing sensor information, will improve the accuracy and reliability of the system. Furthermore, the path planning algorithm will be optimized to adapt to dynamic environments, effectively avoiding moving obstacles and reducing computation time. Control performance will be enhanced by implementing adaptive control techniques and reinforcement learning, enabling smoother and more precise vehicle movements. Finally, real world testing in diverse parking environments will be conducted to evaluate the system's effectiveness and explore cost-reduction solutions for potential commercialization.

