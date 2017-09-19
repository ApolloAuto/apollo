# Method for Calibrating Extrinsic Parameters Between Multiple-beam LiDAR and GNSS/INS

## Introduction

In autonomous driving, combining multiple-beam LiDAR and GNSS/INS is a popular sensor configuration for high definition map production, point-cloud-based localization and point-cloud-based object detection. A precise calibration of the extrinsic parameters is necessary for a successful implementation. 

In many applications, LiDAR is placed at the top of a vehicle horizontally to acquire 360 degrees field of view, while IMU is aligned with the vehicle axis to simplify the coordinate transformation, as shown in Figure 1. Apollo calibration service is mainly optimized for this type of installation.

![](images/lidar_calibration/install.png)

<center>Figure 1. A typical sensor installation. Red circle represents the top-mounted LiDAR and the square is the IMU.</center>

## Method

In this section, we will introduce the basics of calibrating LiDAR and GNSS/INS.

#### 1. Calculate the initial value of extrinsic parameters by hand-eye calibration

As there is no direct correspondence between LiDAR measurements and GNSS/INS measurements, we need to solve initial value by hand-eye calibration. 

For example, assuming that the GNSS/INS pose at time $t_i$ is $T_i^{ins}$ , while at the same time, LiDAR pose from LiDAR odometry or SLAM algorithm is $T_i^{lidar}$, the classic hand-eye calibration is defined as solving $T^{ins}_{lidar}$, such that:
$$T_{i,i+1}^{ins}T_{lidar}^{ins}=T_{lidar}^{ins}T_{i,i+1}^{lidar}, \tag{1}$$
where $T_{i,i+1}^{ins}=T_{i+1}^{ins}(T_{i}^{ins})^{-1}$, $T_{i,i+1}^{lidar}=T_{i+1}^{lidar}(T_{i}^{lidar})^{-1}$ are the relative motions of the two sensors, respectively. As the vehicle motion is approximately planar, the problem can be simplified as follow:
$$(R_{ins}-I)t=Rt_{lidar}-t_{ins}, \tag{2}$$
where $R_{ins}$ and $t_{ins}$ are the rotation and translation of the relative motion of GNSS/INS, $t_{lidar}$ is the translation of LiDAR, respectively, $R$ and $t$ form the extrinsic parameters. Let 
$$R=\left(\begin{matrix} cos\theta & -sin\theta\\sin\theta & cos\theta \end{matrix}\right),$$ 
then Equation (2) becomes:
$$
\left(\begin{array}{c|c} R_{ins}-I & \begin{matrix} -t_{lidar}^x & t_{lidar}^y \\ -t_{lidar}^y & -t_{lidar}^x \end{matrix} \end{array}\right)
\left(\begin{matrix} t^x\\t^y\\cos\theta\\sin\theta\end{matrix}
\right)=-t_{ins}. \tag{3}$$
From Equation (3) we know, a single relative motion places two constraints on four unknowns, so if we have three different motions, the equation is full rank, thus can be solved linearly. In order to collect sufficient constraints when solving initial value and optimizing point cloud alignment in the following section, the algorithm needs the vehicle be driven following the trajectory which looks like ‘8’, as shown in Figure 2. 

**Note:** as the vehicle motion is approximately planar, the height between LiDAR and IMU cannot be reliably determined.

![](images/lidar_calibration/trajectory.png)

<center>Figure 2. The trajectory needed for calibration.</center>

#### 2. Point cloud registration based extrinsic parameters optimization

Using the pose from GNSS/INS and initial estimation of extrinsic parameters, the registration of point clouds captured at different places can be conducted. As the initial value is not precise, registration error can be found in the registration point cloud. The error makes the point cloud lack of details and edges blurred. So, the second step of our algorithm is optimizing extrinsic parameters by improving point cloud registration quality. A typical GICP or Entropy cost can be used in this optimization process.

