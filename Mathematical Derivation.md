# Mathematical Derivation 

## 1. State Transition

### 1. Differential Robot Control Model

$v = \frac{r\dot{\phi_r}}{2} + \frac{r\dot{\phi_l}}{2} $

$\omega = \frac{r\dot{\phi_r}}{b} - \frac{r\dot{\phi_l}}{b} $

$v$ : linear velocity along the robot direction 

$\omega$ : angular velocity 

$r$ : robot wheel radius 

$b$ : robot inter-wheel distance 

$\dot{\phi_r} $ and $\dot{\phi_l}$ : right wheel and left wheel angular velocity

### 2. A Priori Estimation of Current State 

$\hat{T_t} =\begin{bmatrix} \hat{x_{t}} \\\hat{y_{t}} \\ \hat{\theta_{t}} \end{bmatrix} = \begin{bmatrix} x_{t-1} \\y_{t-1} \\\theta_{t-1} \end{bmatrix} +   \begin{bmatrix} (\Delta s_l+\Delta s_r)/2  \cdot cos(\theta_{t-1} + \Delta \theta/2)  \\(\Delta s_l+\Delta s_r)/2  \cdot sin(\theta_{t-1} + \Delta \theta/2) \\ \Delta \theta \end{bmatrix}$

$\hat{T_t} =\begin{bmatrix} \hat{x_{t}} \\ \hat{y_{t}} \\ \hat{\theta_{t}} \end{bmatrix} = \begin{bmatrix} x_{t-1} \\y_{t-1} \\\theta_{t-1} \end{bmatrix} +   \begin{bmatrix} (\Delta s_l+\Delta s_r)/2  \cdot cos(\theta_{t-1} + (\Delta s_r-\Delta s_l)/2b))  \\(\Delta s_l+\Delta s_r)/2  \cdot sin(\theta_{t-1} + (\Delta s_r-\Delta s_l)/2b)) \\(\Delta s_r-\Delta s_l)/b \end{bmatrix}$

**Command:** $u = \begin{bmatrix} \Delta s_l \\ \Delta s_r \end{bmatrix}$

**Robot pose: ** $T_t = \begin{bmatrix} x_t \\ y_t \\ \theta_t\end{bmatrix}$ 

$\Delta s_r $ and $\Delta s_l $ : move length of right wheel and left wheel

$\Delta s_r = \Delta t \cdot r \dot{\phi_r}$  

$\Delta s_l = \Delta t \cdot r \dot{\phi_l} $ 

In SLAM problem, we need to combine robot pose and map entries to single state vector, and estimate both pose and map entries at the same time. The whole state vector can be expressed as 

$x_t = \begin{bmatrix} x_t & y_t & \theta_t & \alpha_t^1 & r_t^1 & \alpha_t^2 & r_t^2 & \dots & \alpha_t^m & r_t^m\end{bmatrix}^T$

there are $m$ map entries in the map 

### 3. Jacobians of State Transition Model  

#### 1. Jacobian with respect to State

$F_x = \begin{bmatrix} 1 & 0 & -(\Delta s_l + \Delta s_r)/2 \cdot sin(\theta_{t-1} + (\Delta s_r-\Delta s_l)/2b) \\ 0 & 1 & (\Delta s_l + \Delta s_r)/2 \cdot cos(\theta_{t-1} + (\Delta s_r-\Delta s_l)/2b)  & & \large{0} \\ 0 & 0 & 1 \\ &  & \large{0} & & \large{I} \\ \end{bmatrix}$  

#### 2. Jacobian with respect to Control Input

$F_u = \begin{bmatrix} \frac{1}{2} \cdot cos(\theta_{t-1} + \Delta \theta /2) + \frac{1}{2b} \cdot \Delta s \cdot sin(\theta_{t-1} + \Delta \theta /2) & \frac{1}{2} \cdot cos(\theta_{t-1} + \Delta \theta /2) - \frac{1}{2b} \cdot \Delta s \cdot sin(\theta_{t-1} + \Delta \theta /2)  \\ \frac{1}{2} \cdot sin(\theta_{t-1} + \Delta \theta /2) - \frac{1}{2b} \cdot \Delta s \cdot cos(\theta_{t-1} + \Delta \theta /2)  & \frac{1}{2} \cdot sin(\theta_{t-1} + \Delta \theta /2) + \frac{1}{2b} \cdot \Delta s \cdot cos(\theta_{t-1} + \Delta \theta /2) \\ -\frac{1}{b} &  \frac{1}{b} \\ 0 & 0\\ \vdots & \vdots \\ 0 & 0\end{bmatrix}$ where: 

$\Delta \theta = (\Delta s_r - \Delta s_l)/b$ 

$\Delta s = (\Delta s_l + \Delta s_r)/2$

### 4. A Priori Estimation of State Covariance 

$\hat{P_t} = F_x \cdot P_{t-1} \cdot F_x^T + F_u \cdot Q_t \cdot F_u^T$

where $Q_t$ is the covariance matrix of the Gaussian noise of state transition motion model:

$Q_t = \begin{bmatrix} k|\Delta s_l| & 0 \\ 0 & k|\Delta s_r|\end{bmatrix}$ 

## 2. Measurement Prediction 

### 1. Observation Model  

Given prior estimate of state, compute predicted measurement 

**Given One landmark, prior estimation of state, compute predicted measurement of that landmark and corresponding Jacobian:**

landmark : $m^i =\begin{bmatrix} \alpha_w^i \\ r_w^i \end{bmatrix}$

 The landmark is expressed in world frame, but the predicted measurement is expressed in laser frame

The predicted observation $\hat{z_t^i}$ in **laser frame** of single landmark $m^i$ in **world frame** for the robot in the prior estimation state $\hat{x_t}$  

$\hat{z_t^i} = \begin{bmatrix} \hat{\alpha_t^i} \\ \hat{r_t^i}\end{bmatrix}= \begin{bmatrix}  \alpha_w^i - \hat{\theta_t} \\ r_w^i - \hat{x_t} \cdot cos(\alpha_w^i) -  \hat{y_{t}} \cdot sin(\alpha_w^i)  \end{bmatrix}$

But the $\alpha$ of $\hat{z_t^j}$ could be $[- \infin, -\pi)$ or $(\pi, \infin]$, and $r$ could be negative, we need to further modify the predicted measurement to make $\hat{\alpha_t^j} \in [-\pi,\pi]$ and $\hat{r_t^j} \geq 0$ 

**First:** 

If $\hat{r_t^j} \leq 0$ : 

​          $\hat{\alpha_t^j} = \hat{\alpha_t^j} + \pi $ 

​	  $\hat{r_t^j} = - \hat{r_t^j}$ 

since we changed the sign of $\hat{r_t^j}$, the corresponding entries in Jacobian matrix of $\hat{z_t^j}$ should change the sign too 

**Second: **

If $\hat{\alpha_t^j} \geq \pi$: 

​        $\hat{\alpha_t^j} = \hat{\alpha_t^j} - 2\pi $

If $\hat{\alpha_t^j} \leq -\pi $

​        $\hat{\alpha_t^j} = \hat{\alpha_t^j}+2\pi ​$ 

the change for $\hat{\alpha_t^j}$ doesn't affect the derivative, so we don't have to change the entries in Jacobian 

### 2. Jacobian of Observation Model 

$H^j = \begin{bmatrix} 0 & 0 & -1 & \dots& 0 & 0 & \dots & 1 & 0 & \dots & 0 & 0 & \dots \\ -cos(\alpha_w^i) & -sin(\alpha_w^i) & 0 & \dots& 0 & 0 & \dots & \hat{x_t} \cdot sin(\alpha_w^i) - \hat{y_t} \cdot cos(\alpha_w^i) & 1 & \dots &0 & 0 & \dots\end{bmatrix}$  

Notice the entries in $H^j$ of $\hat{r_t^i}$ need to change the sign if $r<0$ because we change the sign if $r<0$ , namely, if $r<0$ : $H^j$  will be:

$H^j = \begin{bmatrix} 0 & 0 & -1 & \dots& 0 & 0 & \dots & 1 & 0 & \dots & 0 & 0 & \dots \\ cos(\alpha_w^i) & sin(\alpha_w^i) & 0 & \dots& 0 & 0 & \dots & -\hat{x_t} \cdot sin(\alpha_w^i) + \hat{y_t} \cdot cos(\alpha_w^i) & -1 & \dots &0 & 0 & \dots\end{bmatrix}$  

## 3. Measurement Association 

We need to associate every observed measurement $z_t^j$ with a predicted measurements $\hat{z_t^i}$ . And we need to care about following things:

1. Compute the difference between every observed measurement and every predicted measurement of landmarks 
2. Corrupted measurements: measurements that do not corresponding to any entries in the map. The corrupted measurements will just be abandoned.

**Innovation as difference between a observed measurement and a predicted measurement ** 

$v_t^{ij} = z_t^j - \hat{z_t^i}$ 

**Innovation Covariance ** 

$\Sigma_{IN_t}^{ij} = H^i \cdot \hat{P} \cdot H^{iT} + R^j$ 

**Mahalanobis distance ** 

$d_{ij} = v^{ijT} \cdot \Sigma_{IN_t}^{ij} \cdot v^{ij}$ 

## 4. Compute Posterior Estimation 

### 1. Compute Kalman Gain 

$K_t = \hat{P_t} \cdot H_t^T \cdot(H_t \cdot \hat{P_t} \cdot H_t^T + R_t)^{-1}$   

where $H_t$ is stacked(vertically) measurement prediction covariance 

$H_t = \begin{bmatrix} H_t^1 \\ H_t^2 \\ \cdot \\ \cdot \\ H_t^n\end{bmatrix}$  , $n$ is the number of successfully associated observation and prediction 

$H_t$ is of size $[2n,3+2m]$ 

$R_t$ is block diagonalized observation covariance matrix of size $[2n,2n]$ 

The size of $K_t$ is $[3+2m,2n]$  

### 2. Apply Kalman filter and compute posterior estimation 

$x_t = \hat{x_t} + K_t (z_t - \hat{z_t})$ 

$P_t = (I-K_tH_t)\hat{P_t}$ 

$z_t$ and $\hat{z_t}$ is stacked observations and predicted measurements, they are of size $[2n,1]$ 

## 5. Add New map entries, Uncertainty propagation 

When we encounter an unassociated observation, we add the observation as new map entry in the map. 

### 1. Convert observation from  Laser frame to world frame

For unassociated observation in laser frame $z_l^i = \begin{bmatrix} \alpha_l^i \\ r_l^i \end{bmatrix}$

Compute the inverse transformation from laser frame to world frame. 

The current posterior estimation of laser frame in world frame is $T = \begin{bmatrix} x_t \\ y_t \\ \theta_t \end{bmatrix}$

$T_{lw} = \begin{bmatrix} x_t^{lw} \\ y_t^{lw} \\ \theta_t^{lw} \end{bmatrix} = \begin{bmatrix} -cos(\theta_t) & -sin(\theta_t) & 0\\ sin(\theta_t) & -cos(\theta_t) & 0 \\ 0& 0& -1\end{bmatrix} \begin{bmatrix} x_t \\ y_t \\ \theta_t\end{bmatrix}$

The Jacobian matrix of $T_lw$ wrt $T$ is 

$J_{T} = \begin{bmatrix} -cos(\theta_t) & -sin(\theta_t) & x_t \cdot sin(\theta_t) - y_t \cdot cos(\theta_t) \\ sin(\theta_t) & -cos(\theta_t)  & x_t \cdot cos(\theta_t) + y_t \cdot sin(\theta_t) \\ 0& 0& -1\end{bmatrix}$ 

The covariance matrix of $T_{lw}$ is:

$P_{T_{lw}} = J_T P_{T_t} J_T^T$

where $P_{T_t}$ is the posterior estimation covariance matrix of robot pose  

### 2. Uncertainty propagation

Corresponding new map entry $m^i$ in world frame is a function of both $T_{lw}$ and  $z_t^i$, so we need to propagate both observation's uncertainty $R^i$ and inverse transformation's uncertainty $P_{T_{lw}}$ to derive the new map entry's uncertainty $P_{m^i}$ 

$z_t^i = \begin{bmatrix} \alpha_l^i \\ r_l^i \end{bmatrix} $ $l$ means in laser frame

$m_i = \begin{bmatrix} \alpha_w^i \\ r_w^i \end{bmatrix} = \begin{bmatrix} \alpha_l^i -\theta_t^{lw} \\ r_l^i  - x_t^{lw} \cdot  cos(\alpha_l^i) - y_t^{lw} \cdot sin(\alpha_l^i)\end{bmatrix}$  

or:

$m_i = \begin{bmatrix} \alpha_w^i \\ r_w^i \end{bmatrix} = \begin{bmatrix} \alpha_l^i -\theta_t^{lw} \\ -r_l^i  + x_t^{lw} \cdot  cos(\alpha_l^i) + y_t^{lw} \cdot sin(\alpha_l^i)\end{bmatrix}$ 

First we need to calculate the Jacobian matrix of $m^i$ wrt $z_t^i$ and $T_{lw}$, and it is almost the same as the Jacobian matrix we derived in measurement prediction step.

$J_{z} = \begin{bmatrix} 1 & 0 \\ x_t^{lw} \cdot sin(\alpha_l^i) - y_t^{lw} \cdot cos(\alpha_l^i) & 1\end{bmatrix}$

$J_{z} = \begin{bmatrix} 1 & 0 \\ -x_t^{lw} \cdot sin(\alpha_l^i) + y_t^{lw} \cdot cos(\alpha_l^i) & -1\end{bmatrix}$  

We also need to calculate the Jacobian matrix of $m^i$ wrt $T_{lw}$ 



$J_{T_{lw}} = \begin{bmatrix} 0 & 0 & -1 \\ -cos(\alpha_i^i) & -sin(\alpha_l^i) & 0\end{bmatrix}$

$J_{T_{lw}} = \begin{bmatrix} 0 & 0 & -1 \\ cos(\alpha_i^i) & sin(\alpha_l^i) & 0\end{bmatrix}$ 

Then we can calculate covariance matrix of map entry in world frame as follow:

$P_{m^i} = J_z R^i J_z^T + J_{T_{lw}}P_{T_{lw}} J_{T_{lw}}^T = J_z R^i J_z^T + J_{T_{lw}} J_T P_{T_t} J_T^T J_{T_{lw}}^T $





 