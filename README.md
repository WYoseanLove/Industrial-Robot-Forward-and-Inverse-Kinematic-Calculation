# Industrial-Robot-Forward-and-Inverse-Kinematic-Calculation
Robot_Kinematic_Calculation used to calculate the familiar industrial Robot (KUKA, ABB, FANUC, KAWASAKI) 6 DOF forward Kinematic and inverse Kinematic. From the class you can given input J1-J6 joint degree for Robot and will auto calculate the end effector posture, and same if you give the end effector posture, you can get the J1-J6 JOINT degree for all the possible solutions. The calculation is base on c++ Eigen Library, and all the calculation is deducted from J1 until J6.

Usage: 6 DOF Industrial Robot forward Kinematic and inverse Kinematic, can be modified to other industry robot like ABB, FANUC and Kawasaki

References: https://www.youtube.com/watch?v=C0H8XsSPGyo https://www.youtube.com/watch?v=NDEEKGEQylg
Previous repository(https://github.com/WYoseanLove/KUKA-Robot-Forward-Inverse-Kinematic-Calculation-.git ) is only for KUKA Robot, the new repository is updated for normal industrial Robot and optimize the inverse kinematic calculation method, all the possible solutions can be got and also verified in SIEMENS Process simulate
