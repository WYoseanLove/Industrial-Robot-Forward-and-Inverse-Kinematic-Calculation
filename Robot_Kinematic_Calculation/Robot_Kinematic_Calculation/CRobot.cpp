#include <iostream>
#include "CRobot.h"
#include <vector>



bool CRobot::isRotationMatrix(Eigen::Matrix3d R)
{
	double err = 1e-6;

	Eigen::Matrix3d shouldIdenity;
	shouldIdenity = R * R.transpose();

	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();


	bool result = (shouldIdenity - I).norm() < err;
	return result;
}

Eigen::Vector3d CRobot::rotationMatrixToEulerAngles(Eigen::Matrix3d & R)
{
	assert(isRotationMatrix(R));
	double sy = sqrt(R(0, 0)*R(0, 0) + R(1, 0)*R(1, 0));
	bool singlular = sy < 1e-6;
	double x, y, z;
	if (!singlular)
	{
		x = atan2(R(2, 1), R(2, 2));
		y = atan2(-R(2, 0), sy);
		z = atan2(R(1, 0), R(0, 0));


	}
	else
	{
		x = atan2(-R(1, 2), R(1, 1));
		y = atan2(-R(2, 0), sy);
		z = 0;

	}

	return { x,y,z };

}

double CRobot::Robot_Joint_Value_Check(double Joint,double J_Lower, double J_Upper)
{
	double J1_solution = Joint;
	if ((J1_solution >= J_Lower) && (J1_solution <= J_Upper))
		return J1_solution;
	else if ((J1_solution + 360 >= J_Lower) && (J1_solution + 360 <= J_Upper))
		return J1_solution+360;
	else if ((J1_solution - 360 >= J_Lower) && (J1_solution - 360 <= J_Upper))
		return J1_solution - 360;
	else
		return 720;
}

bool CRobot::check_robot_solution(double * solution)
{
	if (solution != NULL)
	{
		for (int i = 0; i < 6; i++)
		{

			if (solution[i] == 720)
			{
				return false;
			}
		}

	}
	
	return true;
}

Eigen::MatrixXd CRobot::KW_Robot(double * a1, double * d1, double * a2, double * a3, double * d4, double * d5, double * TCPX, double * TCPY, double * TCPZ, double * TCPRX, double * TCPRY, double * TCPRZ)
{
	Eigen::MatrixXd RT;
	RT = Eigen::MatrixXd(4, 4);
	/*
	from the Base frame to calculate the J1 Joint location and rotation frame
	from the base frame to J1 frame will be through below steps:
	1. base Frame should be move *D1 along with Z aix direction;
	2. base frame should be move *a1 along with X aix direction;
	3. base frame should be rotate -90 deg  with Z aix;
	4. base frame should be rotate -90 deg with Y aix;
	after 4 steps, the base frame will be at J1 location and same direction with J1;
	*/
	RT = KW_Robot_Joint(*a1,0, *d1,0,-90,-90,(J1 - J1_offset));
	/*
	from the J1 frame to calculate the J2 Joint location and rotation frame
	from the J1 frame to J2 frame will be through below steps:
	1. J1 frame should be move *a2 along with X aix direction;
	2. J1 frame should be rotate 0 deg  with Z aix;
	3. J1 frame should be rotate 0 deg with Y aix;
	4. J1 frame should be rotate 0 deg with X aix;
	after 4 steps, the base frame will be at J1 location and same direction with J1;
	*/
	RT*= KW_Robot_Joint(*a2, 0, 0, 0, 0, 0, -(J2 - J2_offset));
	/*
	from the J2 frame to calculate the J3 Joint location and rotation frame
	from the J2 frame to J3 frame will be through below steps:
	1. J2 frame should be move -*a3 along with X aix direction;
	1. J2 frame should be move *d4 along with Y aix direction;
	2. J2 frame should be rotate 0 deg  with Z aix;
	3. J2 frame should be rotate 0 deg with Y aix;
	4. J2 frame should be rotate -90 deg with X aix;
	after 4 steps, the base frame will be at J1 location and same direction with J1;
	*/
	RT *= KW_Robot_Joint(-*a3, *d4, 0, -90, 0, 0, -(J3 - J3_offset));
	/*
	from the J3 frame to calculate the J4 Joint location and rotation frame
	from the J3 frame to J4 frame will be through below steps:
	1. J3 frame should be move 0 along with X aix direction;
	2. J3 frame should be move 0 along with Y aix direction;
	3. J3 frame should be move 0 along with Z aix direction;
	4. J3 frame should be rotate 0 deg  with Z aix;
	5. J3 frame should be rotate 0 deg with Y aix;
	6. J3 frame should be rotate 90 deg with X aix;
	after 6 steps, the base frame will be at J1 location and same direction with J1;
	*/
	RT *= KW_Robot_Joint(0, 0, 0, 90, 0, 0, (J4 - J4_offset));
	/*
	from the J4 frame to calculate the J5 Joint location and rotation frame
	from the J4 frame to J5 frame will be through below steps:
	1. J4 frame should be move 0 along with X aix direction;
	2. J4 frame should be move *d5 along with Y aix direction;
	3. J4 frame should be move 0 along with Z aix direction;
	4. J4 frame should be rotate 0 deg  with Z aix;
	5. J4 frame should be rotate 180 deg with Y aix;
	6. J4 frame should be rotate -90 deg with X aix;
	after 6 steps, the base frame will be at J1 location and same direction with J1;
	*/
	RT *= KW_Robot_Joint(0, *d5, 0, -90, 180, 0, -(J5 - J5_offset));
	/*
	from the J5 frame to calculate the TCP Joint location and rotation frame
	from the J5 frame to TCP frame will be through below steps:
	1. J5 frame should be move *TCPX along with X aix direction;
	2. J5 frame should be move *TCPY along with Y aix direction;
	3. J5 frame should be move *TCPZ along with Z aix direction;
	4. J5 frame should be rotate *TCPRZ deg  with Z aix;
	5. J5 frame should be rotate *TCPRY deg with Y aix;
	6. J5 frame should be rotate *TCPRX deg with X aix;
	after 6 steps, the base frame will be at J1 location and same direction with J1;
	*/
	RT *= KW_Robot_Joint(*TCPX, *TCPY, *TCPZ, *TCPRX, *TCPRY, *TCPRZ, (J6 - J6_offset));
	std::cout << RT << std::endl;


	Eigen::Matrix3d R;
	R = Eigen::Matrix3d(3, 3);

	R << RT(0, 0), RT(0, 1), RT(0, 2),
		RT(1, 0), RT(1, 1), RT(1, 2),
		RT(2, 0), RT(2, 1), RT(2, 2);
	Eigen::Vector3d Rotation = rotationMatrixToEulerAngles(R);


	double rx = (Rotation[0]) * 180 / M_PI;
	double ry = (Rotation[1]) * 180 / M_PI;
	double rz = (Rotation[2]) * 180 / M_PI;

	double Tx = RT(0, 3);
	double Ty = RT(1, 3);
	double Tz = RT(2, 3);

	//std::cout << Tx << "  " << Ty << "  " << Tz << "  " << rx << "  " << ry << "  " << rz << "  " << std::endl;
	return RT;


}

CRobot::CRobot()
{
	J1_offset = 0;
	J2_offset = 0;
	J3_offset = 0;
	J4_offset = 0;
	J5_offset = 0;
	J6_offset = 0;
}

Eigen::MatrixXd CRobot::RTCP_J6_Matrix()
{
	Eigen::Vector3d rpy_raw;

	rpy_raw << rx, ry, rz;

	rpy_raw = rpy_raw * M_PI / 180;

	Eigen::MatrixXd RX, RY, RZ, R, RTCP;
	RX = Eigen::MatrixXd(3, 3);
	RY = Eigen::MatrixXd(3, 3);
	RZ = Eigen::MatrixXd(3, 3);
	R = Eigen::MatrixXd(3, 3);
	RTCP = Eigen::MatrixXd(4, 4);

	RX << 1, 0, 0,
		0, cos(rpy_raw[0]), -sin(rpy_raw[0]),
		0, sin(rpy_raw[0]), cos(rpy_raw[0]);


	RY << cos(rpy_raw[1]), 0, sin(rpy_raw[1]),
		0, 1, 0,
		-sin(rpy_raw[1]), 0, cos(rpy_raw[1]);

	RZ << cos(rpy_raw[2]), -sin(rpy_raw[2]), 0,
		sin(rpy_raw[2]), cos(rpy_raw[2]), 0,
		0, 0, 1;

	R = RZ * RY*RX;

	RTCP << R(0, 0), R(0, 1), R(0, 2), Tx,
		R(1, 0), R(1, 1), R(1, 2), Ty,
		R(2, 0), R(2, 1), R(2, 2), Tz,
		0, 0, 0, 1;
	/*
	define the TCP frame coordinate according to the base frame；
	from the end effector frame to inverse calculate the J6 Flange frame value;
	*/

	rpy_raw << TCPRX, TCPRY, TCPRZ;
	rpy_raw = rpy_raw * M_PI / 180;
	Eigen::Isometry3d R_Tool = Eigen::Isometry3d::Identity();
	R_Tool = (Eigen::AngleAxisd(rpy_raw[2], Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(rpy_raw[1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rpy_raw[0], Eigen::Vector3d::UnitX()));
	R_Tool.pretranslate(Eigen::Vector3d(TCPX, TCPY, TCPZ));//在这里平移需要放到轴的坐标上去,相对于旋转之前的坐标系方向

	Eigen::MatrixXd RJ_6;
	RJ_6 = Eigen::MatrixXd(4, 4);
	RJ_6 = RTCP * R_Tool.matrix();

	// a11, a12, a13, x0,
	// a21, a22, a23, y0,
	// a31, a32, a33, z0,
	//   0,   0,   0,  1;

	return RJ_6;

	
}

bool CRobot::IK_Robot_Calculation_J1(Eigen::MatrixXd RJ_6)
{
	
	// a11, a12, a13, x0,
	// a21, a22, a23, y0,
	// a31, a32, a33, z0,
	//   0,   0,   0,  1;


	double a11 = RJ_6(0, 0);
	double a12 = RJ_6(0, 1);
	double a13 = RJ_6(0, 2);
	double x0 = RJ_6(0, 3);

	double a21 = RJ_6(1, 0);
	double a22 = RJ_6(1, 1);
	double a23 = RJ_6(1, 2);
	double y0 = RJ_6(1, 3);

	double a31 = RJ_6(2, 0);
	double a32 = RJ_6(2, 1);
	double a33 = RJ_6(2, 2);
	double z0 = RJ_6(2, 3);

	/*
	A01_inverse* RJ_6*A56_inverse =A15

	A01_inverse* RJ_6*A56_inverse=
	a31*c6+a32*s6,                            -s6*a31+a32*c6,                         a33,            z0-1045,
	c6*c1*a11-c6*s1*a21+s6*a12*c1-s1*s6*a22,  -s6*(c1*a11-a21*s1)+c6*(a12*c1-s1*a22), c1*a13-s1*a23,  c1*x0-s1*y0-500,
	(s1*a11+c1*a21)*c6+s6*(a12*s1+a22*c1),   -s6*(s1*a11+c1*a21)+c6*(s1*a12+c1*a22), s1*a13+c1*a23,  s1*x0+c1*y0,
										0,                                        0,             0,            1;

	A15 << -c23 * c4*c5 + s23 * s5,   -c23 * s4,    c23*c4*s5 + s23 * c5,    290 * s5*c23*c4 + 290 * c5*s23 - 55 * c23 + 1275 * s23 + 1300 * c2,
		  s23*c4*c5 + c23 * s5,       s23*s4,      -s23 * c4*s5 + c23 * c5, -290 * s5*s23*c4 +290 * c5*c23 + 55 * s23 + 1275 * c23 - 1300 * s2,
		   -s4 * c5,                    c4,         s4*s5,                    290 * s5*s4,
		   0,                           0,          0,                        1;

	 A15(2,3)/A15(2,2)=290;
	 (s1*x0+c1*y0)/(s1*a13+c1*a23)=290;
	 tanJ1=(290*a23-y0)/(x0-290*a13);
	*/
	/*
	J1 can be calculate one solution according to atan2 function, and other possible solution is J1 Rotate another angle with M_PI+J1;
	*/
	J1 = atan2(((d5) * a23 - y0), (x0 - (d5) * a13));
	J1_Solution.push_back(J1);
	J1_Solution.push_back(J1 + M_PI);

	if (J1_Solution.empty())
		return false;
	else
		return true;


}

bool CRobot::IK_Robot_Calculation_J2(Eigen::MatrixXd RJ_6)
{
	
	// a11, a12, a13, x0,
	// a21, a22, a23, y0,
	// a31, a32, a33, z0,
	//   0,   0,   0,  1;


	double a11 = RJ_6(0, 0);
	double a12 = RJ_6(0, 1);
	double a13 = RJ_6(0, 2);
	double x0 = RJ_6(0, 3);

	double a21 = RJ_6(1, 0);
	double a22 = RJ_6(1, 1);
	double a23 = RJ_6(1, 2);
	double y0 = RJ_6(1, 3);

	double a31 = RJ_6(2, 0);
	double a32 = RJ_6(2, 1);
	double a33 = RJ_6(2, 2);
	double z0 = RJ_6(2, 3);

	// calculate the J5_Location to support J2/J3 Calculation;
	Eigen::Vector3d RJ_6_Location;
	RJ_6_Location << x0, y0, z0;

	Eigen::Vector3d w(0, 0, 1);
	Eigen::MatrixXd RJ_6_Oritation;
	RJ_6_Oritation = Eigen::MatrixXd(3, 3);

	RJ_6_Oritation << a11, a12, a13,
		a21, a22, a23,
		a31, a32, a33;

	Eigen::Vector3d J5_Location;

	J5_Location = RJ_6_Location - (d5)* RJ_6_Oritation*w;
	/*
	 RJ_5 Frame matrix as below :
	-c1 * s23*c4 + s1 * s4, c1*c23, c1*s23*s4 + s1 * c4, 55 * c1*s23 + 1275 * c1*c23 + 500 * c1 - 1300 * c1*s2,
	s1*s23*c4 + c1 * s4, -s1 * c23, c1*c4 - s1 * s23*s4, -55 * s1*s23 - 1275 * s1*c23 - 500 * s1 + 1300 * s1*s2,
	c23*c4, s23, -c23 * s4, -55 * c23 + 1275 * s23 + 1045 + 1300 * c2,
	0, 0, 0, 1;
	J5_Location(0,0)=55 * c1*s23 + 1275 * c1*c23 + 500 * c1 - 1300 * c1*s2;
	J5_Location(1,0)=-55 * s1*s23 - 1275 * s1*c23 - 500 * s1 + 1300 * s1*s2;
	J5_Location(2,0)= -55 * c23 + 1275 * s23 + 1045 + 1300 * c2;
	J1 = (J1)*M_PI / 180;
	55*s23+1275*c23-1300*s2=J5_Location(0,0)/(cos(J1))-500;
	55*c23-1275*s23-1300*c2=-J5_Location(2,0)+1045;

	double a_temp= J5_Location(0, 0) / (cos(J1)) - 500;
	double b_temp = -J5_Location(2, 0) + 1045;
	double c_temp = (55 * 55 + 1275 * 1275 - a_temp * a_temp - b_temp * b_temp - 1300 * 1300) / (2 * 1300);
	*/
	double a_temp = J5_Location(0, 0) / (cos(J1)) - (a1);
	double b_temp = -J5_Location(2, 0) + d1;
	double c_temp = ((a3) * (a3)+(d4) * (d4)-a_temp * a_temp - b_temp * b_temp - (a2) *(a2)) / (2 * (a2));
	/*
	J2 calculation have two different solutions, sqrt can be +/-;
	for the 1st and 2nd solution will be keep sqrt result is  + ;
	because J2 will be calculated bt asin function, as per asin function result, it will have two different solutions.
	one is angle x, another will be M_PI-X.
	here will keep two results, the purpose is to calcualte all the solutions for Robot;

	*/
	for (int i = 0; i < J1_Solution.size(); i++)
	{
		J1 = J1_Solution[i];
		a_temp = J5_Location(0, 0) / (cos(J1)) - (a1);
		b_temp = -J5_Location(2, 0) + d1;
		c_temp = ((a3) * (a3)+(d4) * (d4)-a_temp * a_temp - b_temp * b_temp - (a2) *(a2)) / (2 * (a2));
		// possible solution 1; 
		if (a_temp > 0)
		{
			double temp = c_temp / (sqrt(a_temp*a_temp + b_temp * b_temp));
			if (abs(temp)<=1)
			{
				
				J2 = asin(c_temp / (sqrt(a_temp*a_temp + b_temp * b_temp))) - atan2(b_temp, a_temp);
				J2_Solution.push_back(J2);
				J2 = asin(c_temp / (-sqrt(a_temp*a_temp + b_temp * b_temp))) - atan2(b_temp, a_temp);
				J2_Solution.push_back(J2);


				J2 = M_PI-asin(c_temp / (sqrt(a_temp*a_temp + b_temp * b_temp))) - atan2(b_temp,a_temp);
				J2_Solution.push_back(J2);
				J2 = M_PI - asin(c_temp / (-sqrt(a_temp*a_temp + b_temp * b_temp))) - atan2(b_temp, a_temp);
				J2_Solution.push_back(J2);

			}
			
			
			
		}

		else
		{
			double temp = -c_temp / (sqrt(a_temp*a_temp + b_temp * b_temp));
			if (abs(temp) <= 1)
			{
				J2 = asin(-c_temp / (sqrt(a_temp*a_temp + b_temp * b_temp))) - atan2(b_temp, a_temp);
				J2_Solution.push_back(J2);
				J2 = asin(-c_temp / (-sqrt(a_temp*a_temp + b_temp * b_temp))) - atan2(b_temp, a_temp);
				J2_Solution.push_back(J2);

				J2 = M_PI-asin(-c_temp / (sqrt(a_temp*a_temp + b_temp * b_temp))) - atan2(b_temp, a_temp);
				J2_Solution.push_back(J2);
				J2 = M_PI - asin(-c_temp / (-sqrt(a_temp*a_temp + b_temp * b_temp))) - atan2(b_temp, a_temp);
				J2_Solution.push_back(J2);

			}
		
		}


	}


	if (J2_Solution.empty())
		return false;
	else
		return true;
	
	
}

bool CRobot::IK_Robot_Calculation_J3(Eigen::MatrixXd RJ_6)
{
	// a11, a12, a13, x0,
	// a21, a22, a23, y0,
	// a31, a32, a33, z0,
	//   0,   0,   0,  1;


	double a11 = RJ_6(0, 0);
	double a12 = RJ_6(0, 1);
	double a13 = RJ_6(0, 2);
	double x0 = RJ_6(0, 3);

	double a21 = RJ_6(1, 0);
	double a22 = RJ_6(1, 1);
	double a23 = RJ_6(1, 2);
	double y0 = RJ_6(1, 3);

	double a31 = RJ_6(2, 0);
	double a32 = RJ_6(2, 1);
	double a33 = RJ_6(2, 2);
	double z0 = RJ_6(2, 3);

	// calculate the J5_Location to support J2/J3 Calculation;
	Eigen::Vector3d RJ_6_Location;
	RJ_6_Location << x0, y0, z0;

	Eigen::Vector3d w(0, 0, 1);
	Eigen::MatrixXd RJ_6_Oritation;
	RJ_6_Oritation = Eigen::MatrixXd(3, 3);

	RJ_6_Oritation << a11, a12, a13,
		a21, a22, a23,
		a31, a32, a33;

	Eigen::Vector3d J5_Location;

	//J5_Location = TCP - 290 * TCP_Oritation*w;
	J5_Location = RJ_6_Location - (d5)* RJ_6_Oritation*w;
	/*
	 RJ_5 Frame matrix as below :
	-c1 * s23*c4 + s1 * s4, c1*c23, c1*s23*s4 + s1 * c4, 55 * c1*s23 + 1275 * c1*c23 + 500 * c1 - 1300 * c1*s2,
	s1*s23*c4 + c1 * s4, -s1 * c23, c1*c4 - s1 * s23*s4, -55 * s1*s23 - 1275 * s1*c23 - 500 * s1 + 1300 * s1*s2,
	c23*c4, s23, -c23 * s4, -55 * c23 + 1275 * s23 + 1045 + 1300 * c2,
	0, 0, 0, 1;
	J5_Location(0,0)=55 * c1*s23 + 1275 * c1*c23 + 500 * c1 - 1300 * c1*s2;
	J5_Location(1,0)=-55 * s1*s23 - 1275 * s1*c23 - 500 * s1 + 1300 * s1*s2;
	J5_Location(2,0)= -55 * c23 + 1275 * s23 + 1045 + 1300 * c2;
	J1 = (J1)*M_PI / 180;
	55*s23+1275*c23-1300*s2=J5_Location(0,0)/(cos(J1))-500;
	55*c23-1275*s23-1300*c2=-J5_Location(2,0)+1045;

	double a_temp= J5_Location(0, 0) / (cos(J1)) - 500;
	double b_temp = -J5_Location(2, 0) + 1045;
	double c_temp = (55 * 55 + 1275 * 1275 - a_temp * a_temp - b_temp * b_temp - 1300 * 1300) / (2 * 1300);

	because J3 will be calculated bt asin function, as per asin function result, it will have two different solutions.
	one is angle x, another will be M_PI-X.
	here will keep two results, the purpose is to calcualte all the solutions for Robot;


	J1, J2, J3 is the 1st calculate phase for Robot inverse calcualtion, and almost 8 solutions will be calculate becasue of asin fuction ;

	here will be use J5 location to verify which solution is correct, for the wrong solutions will be  by-pass, and no need to calculate for J4, J5 and J6;

	confirm_J5_Location() is the function to verification;
	*/

	for (int i = 0; i < J1_Solution.size(); i++)
	{
		J1 = J1_Solution[i];
		double a_temp = J5_Location(0, 0) / (cos(J1)) - (a1);
		double b_temp = -J5_Location(2, 0) + d1;
		double c_temp = ((a3) * (a3)+(d4) * (d4)-a_temp * a_temp - b_temp * b_temp - (a2) *(a2)) / (2 * (a2));

		for (int j = 0; j < J2_Solution.size() / 2; j++)
		{
			if (i == 0)
			{
				
				J2 = J2_Solution[j];
				J1 = J1_Solution[i];
				J3 = asin(((a3)*(a_temp + (a2)*sin(J2)) - (d4)*(b_temp + (a2)*cos(J2))) / ((a3) *(a3)+(d4) * (d4))) - J2;
				bool J5_Location_Confirm = confirm_J5_Location(J5_Location,J1, J2, J3);
				if (!J5_Location_Confirm)
					J3 = asin(M_PI);
				J3_Solution.push_back(J3);
				J3 = M_PI-asin(((a3)*(a_temp + (a2)*sin(J2)) - (d4)*(b_temp + (a2)*cos(J2))) / ((a3) *(a3)+(d4) * (d4))) - J2;
				J5_Location_Confirm = confirm_J5_Location(J5_Location,J1, J2, J3);
				if (!J5_Location_Confirm)
					J3 = asin(M_PI);
				J3_Solution.push_back(J3);
				
			}
			else
			{
				
				J2 = J2_Solution[j+ J2_Solution.size() / 2];
				J1 = J1_Solution[i];
				
				J3 = asin(((a3)*(a_temp + (a2)*sin(J2)) - (d4)*(b_temp + (a2)*cos(J2))) / ((a3) *(a3)+(d4) * (d4))) - J2;
				bool J5_Location_Confirm = confirm_J5_Location(J5_Location, J1, J2, J3);
				if (!J5_Location_Confirm)
					J3 = asin(M_PI);
				J3_Solution.push_back(J3);
				J3 = M_PI - asin(((a3)*(a_temp + (a2)*sin(J2)) - (d4)*(b_temp + (a2)*cos(J2))) / ((a3) *(a3)+(d4) * (d4))) - J2;
				J5_Location_Confirm = confirm_J5_Location(J5_Location, J1, J2, J3);
				if (!J5_Location_Confirm)
					J3 = asin(M_PI);
				J3_Solution.push_back(J3);

			}

		}





	}
	
	if (J3_Solution.empty())
		return false;
	else
		return true;
}

std::vector<double> CRobot::IK_Robot_Calculation_J4_J5_J6(double J1, double J2, double J3, Eigen::MatrixXd RJ_6)
{
	double c1 = cos(J1);
	double s1 = sin(J1);
	double c2 = cos(J2);
	double s2 = sin(J2);
	double c3 = cos(J3);
	double s3 = sin(J3);


	Eigen::MatrixXd A01;
	A01 = Eigen::MatrixXd(4, 4);

	A01 << 0, c1, s1, (a1) * c1,
		0, -s1, c1, -(a1) * s1,
		1, 0, 0, (d1),
		0, 0, 0, 1;

	Eigen::MatrixXd A12;
	A12 = Eigen::MatrixXd(4, 4);

	A12 << c2, s2, 0, (a2) * c2,
		-s2, c2, 0, -(a2) * s2,
		0, 0, 1, 0,
		0, 0, 0, 1;

	Eigen::MatrixXd A23;
	A23 = Eigen::MatrixXd(4, 4);

	A23 << c3, 0, s3, -(a3) * c3 + (d4) * s3,
		-s3, 0, c3, (a3) * s3 + (d4) * c3,
		0, -1, 0, 0,
		0, 0, 0, 1;


	Eigen::MatrixXd A03;
	A03 = Eigen::MatrixXd(4, 4);

	A03 = A01 * A12*A23;
	Eigen::MatrixXd A36;
	A36 = Eigen::MatrixXd(4, 4);

	A36 = (A03.inverse())*RJ_6;
	/*
	A36=A34*A45*A56;
	A36<< -c4*c5*c6+s4*s6,   -c4*c5*s6-s4*c6,  c4*s5,  (*d5)*c4*s5,
		   s4*c5*c6+c4*s6,   s4*c5*s6-c4*c6,   -s4*s5, -(*d5)*s4*s5,
		   s5*c6         ,   s5*s6,            c5,     (*d5)*c5,
			   0,                 0,            0,      1;
	*/

	J4 = atan2(-A36(1, 2), A36(0, 2));
	J5 = atan2(sqrt(A36(0, 2)*A36(0, 2) + A36(1, 2)*A36(1, 2)), A36(2, 2));
	J6 = atan2(A36(2, 1), A36(2, 0));

	std::vector<double> J4_J5_J6;
	J4_J5_J6.push_back(J4);
	J4_J5_J6.push_back(J5);
	J4_J5_J6.push_back(J6);


	J4 = atan2(-A36(1, 2), A36(0, 2))+M_PI;
	J5 = atan2(-sqrt(A36(0, 2)*A36(0, 2) + A36(1, 2)*A36(1, 2)), A36(2, 2));
	J6 = atan2(A36(2, 1), A36(2, 0)) + M_PI;

	J4_J5_J6.push_back(J4);
	J4_J5_J6.push_back(J5);
	J4_J5_J6.push_back(J6);



	return J4_J5_J6;
}

bool CRobot::confirm_J5_Location(Eigen::Vector3d J5_Location,double J1, double J2, double J3)
{
	
	Eigen::Vector3d J5_Location_Confirm;
	/*
	J5_Location(0, 0) = 55 * c1*s23 + 1275 * c1*c23 + 500 * c1 - 1300 * c1*s2;
	J5_Location(1, 0) = -55 * s1*s23 - 1275 * s1*c23 - 500 * s1 + 1300 * s1*s2;
	J5_Location(2, 0) = -55 * c23 + 1275 * s23 + 1045 + 1300 * c2;
	*/
	J5_Location_Confirm(0, 0) = a3 * cos(J1)*sin(J2 + J3) + d4 * cos(J1)*cos(J2 + J3) + a1 * cos(J1) - a2 * cos(J1)*sin(J2);
	J5_Location_Confirm(1, 0) = -a3 * sin(J1)*sin(J2 + J3) - d4 * sin(J1)*cos(J2 + J3) - a1 * sin(J1) + a2 * sin(J1)*sin(J2);
	J5_Location_Confirm(2, 0)= -a3 * cos(J2 + J3) + d4 * sin(J2 + J3)+ a2 * cos(J2)+d1;

	if ((abs(J5_Location_Confirm(0, 0) - J5_Location(0, 0)) < 0.05) && (abs(J5_Location_Confirm(1, 0) - J5_Location(1, 0)) < 0.05) && (abs(J5_Location_Confirm(2, 0) - J5_Location(2, 0)) < 0.05))

		return true;
	else
		return false;

}

Eigen::MatrixXd CRobot::KW_Robot_Joint(double  X_offset, double  Y_offset, double  Z_offset,double alpha, double beta, double gamma,double Joint)
{
	/*
	 a1,  d1,  a2,  a3,  d4,  d5 is the Robot DH parameters;
	TCPX, TCPY, TCPZ, TCPRX, TCPRY,  TCPRZ is Tool end effector flange frame to TCP frame, TCPRX, TCPRY, TCPRZ shoud be deg not radian

	* X_offset is the data offset value in X direction from base coordinate to joint coordinate which need to be calculated ;

	*/
	Eigen::MatrixXd RT;
	RT = Eigen::MatrixXd(4, 4);
	/*
	from the Base frame to calculate the J1 Joint location and rotation frame
	from the base frame to J1 frame will be through below steps:
	1. base Frame should be move Z_offset along with Z aix direction;
	2. base Frame should be move Y_offset along with Y aix direction;
	3. base frame should be move X_offset along with X aix direction;
	4. base frame should be rotate gamma deg  with Z aix firstly;
	5. base frame should be rotate beta deg with Y aix after last rotation;
	6. base frame should be rotate alpha deg with X aix after last rotation;
	7. "Joint" VALUE is the base Joint itself rotate degree which is decided by the user;
	after 4 steps, the base frame will be at J1 location and same direction with J1;
	*/

	Eigen::Vector3d rpy_raw, ypr;
	//rpy_raw << 0, -90, -90;
	rpy_raw << alpha, beta, gamma;
	rpy_raw = rpy_raw * M_PI / 180;
	Eigen::Isometry3d R_Joint1 = Eigen::Isometry3d::Identity();
	R_Joint1 = (Eigen::AngleAxisd(rpy_raw[2], Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(rpy_raw[1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rpy_raw[0], Eigen::Vector3d::UnitX()));
	R_Joint1.pretranslate(Eigen::Vector3d((X_offset), Y_offset, (Z_offset)));//

	// If J1 rotate deg, what is the final J1 martix, can be got from below calculation：

	Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
	T1.rotate((Eigen::AngleAxisd((Joint)*M_PI / 180, Eigen::Vector3d::UnitZ())).toRotationMatrix());
	T1.pretranslate(Eigen::Vector3d(0, 0, 0));

	RT = (T1.matrix()).inverse()*(R_Joint1.matrix());

	
	return RT;
}

std::vector<Joint_solution> CRobot::IK_Calculate_Result()
{
	// print J1-J6 as the calculation result for Robot inverse kinematic;
	std::vector<Joint_solution> results;
	J1 = 0;
	J2 = 0;
	J3 = 0;
	J4 = 0;
	J5 = 0;
	J6 = 0;
	/*
	 End effector TCP to End effector flange frame;
	 The translation matrix is defined by the end effector, need calculate the flange frame from the TCP;
	 flange frame will be same the Robot J6 frame in flange;
	 TCP need to translate to flange frame after the direction is common between TCP/ FLANGE Frame;
	 the example showing need to move X=-502, Y=0, Z=905.61, all the data is according to the original TCP aix;
	 Rotation will be happened after translation happened;
	 TCP rotate Z aix 90 deg;
	 then Rotate Y aix 0 deg;
	 finally rotate X aix 90 deg;
	 the TCP frame will be same direction with Tool flange frame;
	*/

	Eigen::MatrixXd RJ_6= RTCP_J6_Matrix();
	if (!IK_Robot_Calculation_J1(RJ_6))
	{
		std::cout << "J1 calcuation failed !!!" << std::endl;
		return results;
	}

	if (!IK_Robot_Calculation_J2(RJ_6))
	{
		std::cout << "J2 calcuation failed !!!" << std::endl;
		return results;
	};
	if (!IK_Robot_Calculation_J3(RJ_6))
	{
		std::cout << "J3 calcuation failed !!!" << std::endl;
		return results;
	};

	std::vector<double> J2_Solution_temp;
	for (int i = 0; i < J3_Solution.size()-2; i++)
	{
		if (i < J3_Solution.size() / 2-1)
		{
			J1_Solution.insert(J1_Solution.end() - 1, J1_Solution[0]);
			

		}
		else
		{
			J1_Solution.insert(J1_Solution.end() - 1, J1_Solution[J1_Solution.size()-1]);
		}

		


	}

	for (int i = 0; i < J3_Solution.size(); i++)
	{
		if (i % 2 == 0)
			J2_Solution_temp.push_back(J2_Solution[i/2]);
		else
			J2_Solution_temp.push_back(J2_Solution[(i - 1)/2]);


	}

	J2_Solution = J2_Solution_temp;


	for (int i = 0; i < J1_Solution.size(); i++)
	{

		if (isnan(J3_Solution[i]))
		{
			continue;

		}

		else
		{
			std::vector<double> J4_J5_J6 = IK_Robot_Calculation_J4_J5_J6(J1_Solution[i], J2_Solution[i], J3_Solution[i], RJ_6);

			Joint_solution J_result;

			J_result.j1 = J1_Solution[i];
			J_result.j2 = J2_Solution[i];
			J_result.j3 = J3_Solution[i];
			J_result.j4 = J4_J5_J6[0];
			J_result.j5 = J4_J5_J6[1];
			J_result.j6 = J4_J5_J6[2];

			results.push_back(J_result);
			J_result.j1 = J1_Solution[i];
			J_result.j2 = J2_Solution[i];
			J_result.j3 = J3_Solution[i];
			J_result.j4 = J4_J5_J6[3];
			J_result.j5 = J4_J5_J6[4];
			J_result.j6 = J4_J5_J6[5];

			results.push_back(J_result);

		}

	}
	for (int i = 0; i < results.size(); i++)
	{

		if (i < results.size()/2)
		{
			results[i].j1= results[i].j1* 180 / M_PI;
			results[i].j2 = (-(results[i].j2) * 180 / M_PI) + J2_offset;
			results[i].j3 = (-(results[i].j3) * 180 / M_PI) + J3_offset;
			results[i].j4 = results[i].j4 * 180 / M_PI;
			results[i].j5 = (-results[i].j5 * 180 / M_PI);
			results[i].j6 = (results[i].j6 * 180 / M_PI);


		}
		else
		{

			results[i].j1 = results[i].j1 * 180 / M_PI;
			results[i].j2 = (-(results[i].j2) * 180 / M_PI) + J2_offset;
			results[i].j3 = (-(results[i].j3) * 180 / M_PI) + J3_offset;
			//results[i].j4 = (2*M_PI-results[i].j4) * 180 / M_PI;
			results[i].j4 = results[i].j4 * 180 / M_PI;
			results[i].j5 = (-results[i].j5 * 180 / M_PI);
			results[i].j6 = (results[i].j6 * 180 / M_PI);



		}




	}


	
	
	return results;
}
