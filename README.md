# robot-assisted-retraction
Iterative enhanced variable admittance control and null-space optimization


# Prerequisites
* MATLAB
* Kuka Sunrise Toolbox  https://github.com/Modi1987/KST-Kuka-Sunrise-Toolbox 
* NetFT  https://github.com/CameronDevine/NetFT

# Structure of the code
* ATISensor.m
  This is matlab function, and used to read force/torque data from ATI sensors. It is worth noting that the Python package is used here, and you need to install NetFT using `pip install NetFT` in your Python environment.
* ILC-Null.m
  This is the main code, which includes iterative enhanced variable admittance control and null-space optimization. It uses real-time from ATI sensors and velocity loop.
* Admittance.m
  This is the traditional admittance control, and uses the bulit-in joint torque sensor of the KUKA.  
