# Parallel-Parking-Algorithm-for-Ackermann-Robot

A system for accurate, safe, and convenient automated parking, optimizing parking space utilization.

Full report: [link](https://drive.google.com/drive/folders/1Jxh6SIA5WU8RnSLNCc_X52M6VD_sQgNw?usp=sharing)
## 1. Introduction
Recognizing the increasing prevalence of self parking features in modern vehicles, aimed at enhancing accuracy, safety, and driver convenience, this study explores autonomous parking control on mobile robots. This study explores autonomous parking control on mobile robots, offering three key contributions. Firstly, it delivers an efficient platform for research and education, enabling researchers and students to readily grasp relevant concepts, algorithms, and techniques without requiring investment in expensive systems. Furthermore, the study prioritizes safety by utilizing mobile robots for algorithm testing, minimizing collision risks and potential damage. Finally, the robot software system, developed using the Robot Operating System (ROS) and a distributed architecture, ensures high flexibility, scalability, and reusability, facilitating seamless integration and sharing across diverse robotic projects. This study specifically proposes an EKF-based localization approach for automatic parallel parking, utilizing IMU and Encoder data for path planning and tracking. Short-range lidar enhances environmental perception, enabling parking space detection based on point cloud data. Algorithm B Splines algorithm optimizes the parking path, adhering to vehicle kinematic constraints while prioritizing smooth, dynamically feasible maneuvers. The chosen Pure Pursuit algorithm combined with Proportional-Integral controller, valued for its simplicity and stability, ensures reliable tracking with minimized errors.
## 2. Methodology

<p align="center">
  <img src="https://github.com/user-attachments/assets/64251342-e07e-4fbd-830e-832397932c56" />
</p>
<p align="center"> Figure 1. Visualization of the automated parallel parking process</p>


<p align="center">
  <img src="https://github.com/user-attachments/assets/f85e3cd0-5547-4d8c-a7a8-4e9eccae21a3" width="70%" />
</p>
<p align="center"> Figure 2. The Automated Parking System using a UML Sequence Diagram  </p>

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

$$X_k = f(X_{k-1}) + W_{k-1}$$

$$Z_k = h(X_k) + V_k$$

$$\widehat{X_k} = f(X_{k-1})$$

$$P_k = FP_{k-1}F^T$$

$$K = P_kH^T(HP_kH^T + R)^{-1}$$

$$X_k = \widehat{X_k} + K(Z - H\widehat{X_k})$$

$$P_k = (I - KH)P_k(I - KH)^T + KRK^T$$

$$x_k = [x, y, \theta]^T$$

$$u_k = [v, \delta]^T$$



$$h(x_k) = 
\begin{bmatrix}
\theta_k \\
v_k
\end{bmatrix}$$


$$x_{k+1} = f(x_k, u_k) =  
\begin{bmatrix} 
x_k + v_k cos(\theta_k) \Delta t \\ 
y_k + v_k sin(\theta_k) \Delta t \\ 
\theta_k + \frac{v_k}{L} tan(\delta_k) \Delta t
\end{bmatrix}
$$

$$F_k = \frac{\delta f}{\delta x_k} = 
\begin{bmatrix} 
1 & 0 & -v_k sin(\theta_k) \Delta t \\ 
0 & 1 & v_k cos(\theta_k) \Delta t \\ 
0 & 0 & 1
\end{bmatrix}
$$

$$P_k = \frac{\delta f}{\delta u_k} = 
\begin{bmatrix} 
cos(\theta_k) \Delta t & 0 \\ 
sin(\theta_k) \Delta t & 0 \\ 
\frac{tan(\delta_k) \Delta t}{L} & \frac{v_k \Delta t}{L cos^2(\delta_k)}
\end{bmatrix}
$$

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
<p align="center"> Figure 6. Geometric explanation of Pure Pursuit  </p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/5ec4993b-b152-47aa-aa12-df9653088372" />
</p>
<p align="center"> Figure 7. The structure of Pure Pursuit - PI Controller  </p>

$$\frac{sin(2\alpha)}{l_{d}}=\frac{sin(\frac{\pi}{2}-\alpha)}{R}$$

$$\frac{2sin(\alpha)cos(\alpha)}{l_{d}}=\frac{cos(\alpha)}{R}$$

$$\rho=\frac{2sin(\alpha)}{l_{d}}$$

$$\delta_{pp} = tan^{-1}(\frac{L}{R}) = tan^{-1}(\rho L)$$

$$\delta_{pp} = tan^{-1}\left(\frac{2Lsin(\alpha)}{l_{d}}\right)$$

$$\delta_p = K_p e$$

$$\delta_i = K_i \sum e$$

$$\delta(t) = \delta_{pp} + \delta_p + \delta_i = tan^{-1}\left( \frac{2Lsin(\alpha)}{l_d} \right) + K_p e + K_i \sum e$$




## 3. Simulation

<p align="center">
  <img src="https://github.com/user-attachments/assets/85f95f40-0d0f-4d15-830c-cc526bcf5c1e" width="50%" />
</p>
<p align="center"> Figure 8. Parallel parking simulation scene</p>

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


## 4. Experrimental Testing Of Parking Algorithm

| Parameter | Value |
|---|---|
| Wheelbase/m | 0.25 |
| L<sub>f</sub>/m | 0.17 |
| L<sub>r</sub>/m | 0.13 |
| Track/m | 0.22 |
|  δ<sub>max</sub> /rad | 7π/36 | 
| v<sub>max</sub>/ms<sup>-1</sup> | 0.14 |

**TABLE 4. Actual parking space parameters**

<table>
  <thead>
    <tr>
      <th> </th>
      <th colspan="2">Trajectory size (m)</th>
      <th colspan="3">Space size (m)</th>
    </tr>
    <tr>
      <th></th>
      <th>|x<sub>0</sub> - x<sub>1</sub>|</th>
      <th>|y<sub>0</sub> - y<sub>1</sub>|</th>
      <th>d</th>
      <th>Length</th>
      <th>Width</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>1</td>
      <td>0.475</td>
      <td>1.9</td>
      <td>0.65</td>
      <td>0.9</td>
      <td>0.25</td>
    </tr>
    <tr>
      <td>2</td>
      <td>0.425</td>
      <td>1.7</td>
      <td>0.6</td>
      <td>1</td>
      <td>0.25</td>
    </tr>
    <tr>
      <td>3</td>
      <td>0.395</td>
      <td>1.58</td>
      <td>0.57</td>
      <td>1</td>
      <td>0.25</td>
    </tr>
  </tbody>
</table>

**TABLE 5.**

| Case  | Front | Behind | Right Side |
|---|---|---|---|
| 1 |  19 cm | 32 cm | 8.5 cm |
| 2 |  18.5 cm | 31 cm | 8.5 cm |
| 3 |  18 cm | 34 cm | 9.5 cm |


<p align="center">
  <img src="https://github.com/user-attachments/assets/d5e3e598-8097-4631-a32f-60171ea011a0" width="50%" />
</p>
<p align="center"> Figure 12. Block Diagram of the software system structure for a mobile robot</p>


<p align="center">
  <img src="https://github.com/user-attachments/assets/ae225424-696e-40ab-ba63-4a3fb114d565" width="50%" />
</p>
<p align="center"> Figure 13. Hardware architecture of the automated parking system</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/b18b3634-3251-471c-a7ff-51bff424055b" width="70%" />
</p>
<p align="center"> Figure 14. Experimental results in case 1</p>


<p align="center">
  <img src="https://github.com/user-attachments/assets/120fdbd3-5e2e-4f3f-9899-a20a59a34783" width="70%" />
</p>
<p align="center"> Figure 15. Experimental results in case 2</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/f3994010-1528-432d-99d6-3764c0c41f05" width="70%" />
</p>
<p align="center"> Figure 16. Experimental results in case 3</p>


## 5. Conclusion
This study introduces an Automatic Parallel Parking Algorithm for Ackermann-steered robotic vehicles.  The algorithm encompasses three primary tasks: localization, path planning, and trajectory tracking.  Empty parking space detection is achieved by leveraging PointCloud data from an RPlidar sensor in conjunction with geometric algorithms. This method demonstrates accuracy in identifying suitable parking spots, providing crucial input for the subsequent path planning stage.
Departing from traditional Tangent Circle approaches, this research employs a B-Spline trajectory for smoother, more continuous steering commands. This eliminates abrupt transitions between maximum and minimum steering angles, enhancing real-world feasibility.  The chosen control strategy combines a Pure Pursuit controller with a Proportional Integral (PI) component. This combination proves effective in guiding the vehicle along the planned trajectory while mitigating over-reliance on Look-Ahead Distance parameters.
Future development will prioritize enhancing detection and localization capabilities by integrating a camera for lane detection, parking sign recognition, and vehicle identification. This visual data, combined with existing sensor information, will improve the accuracy and reliability of the system. Furthermore, the path planning algorithm will be optimized to adapt to dynamic environments, effectively avoiding moving obstacles and reducing computation time. Control performance will be enhanced by implementing adaptive control techniques and reinforcement learning, enabling smoother and more precise vehicle movements. Finally, real world testing in diverse parking environments will be conducted to evaluate the system's effectiveness and explore cost-reduction solutions for potential commercialization.

