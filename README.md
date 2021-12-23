# StepLengthEstimate
Step Length Estimation Algorithm using IMU

## reference and cite:
more information about the method here: 

Nouriani, A., McGovern, R.A. and Rajamani, R., 2021, May. Step Length Estimation Using Inertial Measurements Units. In 2021 American Control Conference (ACC) (pp. 666-671). IEEE. https://ieeexplore.ieee.org/abstract/document/9483252

Ali Nouriani, Robert A. McGovern, Rajesh Rajamani,Step length estimation with wearable sensors using a switched-gain nonlinear observer,Biomedical Signal Processing and Control,
Volume 69,2021,102822,ISSN 1746-8094,https://doi.org/10.1016/j.bspc.2021.102822.


## integration
simple integration based step length estimation by double integration and zero velocity update in each step in 2D

## angle-based
Using the kinematics and geometry of walking for step length estimation (estimate using the foot tilt angles)

## decentralized branches
ZUPT-based method in 3D with some tweaks for better accuracy and some tested examples. CKF method uses a Constrained Kalman Filter. LMI method uses our nonlinear filter (https://ieeexplore.ieee.org/abstract/document/9483252) to increase the accuracy of step length estimation.
The original method is from openshoe project here: http://www.openshoe.org/?page_id=362
