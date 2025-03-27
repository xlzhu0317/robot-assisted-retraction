# robot-assisted-retraction
Iterative enhanced variable admittance control and null-space optimization


# Prerequisites
* MATLAB
* Kuka Sunrise Toolbox  https://github.com/Modi1987/KST-Kuka-Sunrise-Toolbox 
* NetFT  https://github.com/CameronDevine/NetFT

# Structure of the code
* ATISensor.m
  This is matlab function, and used to read force/torque data from ATI sensors. It is worth noting that the Python package is used here, and you need to install NetFT using `pip install NetFT` in your Python environment.
* Iterativee enhanced control and Null space.m
  This is the main code, which includes iterative enhanced variable admittance control and null-space optimization. It uses real-time from ATI sensors and velocity loop.
* Interaction_model.m
  This is the fixed-parameters admittance control, and uses the bulit-in joint torque sensor of the KUKA.


If this project helps your research or work, please consider citing the following paper:

> **X. Zhu, Y. Jiang, R. He, C. Li, and X. Duan**,  "Development and Validation of a Robot-Assisted Retraction System for Orthopedic Surgery,"  *IEEE Transactions on Medical Robotics and Bionics*, doi: [10.1109/TMRB.2025.3550648](https://doi.org/10.1109/TMRB.2025.3550648).



