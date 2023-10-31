#pragma once
#include<Eigen\Dense>
#include <vector>

#define M_PI 3.1415926
struct Joint_solution
{
	double j1;
	double j2;
	double j3;
	double j4;
	double j5;
	double j6;

};


class CRobot
{
public:
	/*
	rx ry rz tx ty tz is to define the industrial Robot TCP frame, the value of the TCP is according to the base frame of Robot;
	setting the base frame of Robot as working frame, Measure the TCP frame value;
	rx ry rz shoud be deg not radian;
	*/
	double rx = 0;
	double ry = 0;
	double rz = 0;

	double Tx = 0;
	double Ty = 0;
	double Tz = 0;  

	/*
	J1-J6 are industrial Robot 6 axis value;
	J1-J6 shoud be deg not radian;
	J1-J6 should be as the input value to set at Robot controller, different type robot should have additional offet value named as J*_offset;
	j*_offset is the data when the robot is at HOME POSE
	*/


	double J1 = 0;
	double J2 = 0;
	double J3 = 0;
	double J4 = 0;
	double J5 = 0;
	double J6 = 0; 

	/*
	 a1,  d1,  a2,  a3,  d4,  d5 is the Robot DH parameters;
	 TCPX, TCPY, TCPZ, TCPRX, TCPRY,  TCPRZ is Tool end effector flange frame to TCP frame, TCPRX, TCPRY, TCPRZ shoud be deg not radian
	*/


	double a1 = 0;
	double d1 = 0;
	double a2 = 0;
	double a3 = 0;
	double d4 = 0;
	double d5 = 0;
	double TCPX = 0;
	double TCPY = 0;
	double TCPZ = 0;

	double TCPRX = 0;
	double TCPRY = 0;
	double TCPRZ = 0;


	
public:
	double J1_offset = 0;
	double J2_offset = 0;
	double J3_offset = 0;
	double J4_offset = 0;
	double J5_offset = 0;
	double J6_offset = 0;

private:
	std::vector<double> J1_Solution;
	std::vector<double> J2_Solution;
	std::vector<double> J3_Solution;
	std::vector<double> J4_Solution;
	std::vector<double> J5_Solution;
	std::vector<double> J6_Solution;

protected:
	bool isRotationMatrix(Eigen::Matrix3d R);
	Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);
	double Robot_Joint_Value_Check(double Joint, double J_Lower, double J_Upper);
	bool check_robot_solution(double* solution);
	Eigen::MatrixXd KW_Robot_Joint(double  X_offset, double  Y_offset, double  Z_offset, double alpha, double beta, double gamma, double Joint);

public:
	Eigen::MatrixXd KW_Robot(double* a1, double* d1, double* a2, double* a3, double* d4, double* d5, double* TCPX, double* TCPY, double* TCPZ, double* TCPRX, double* TCPRY, double* TCPRZ);
	std::vector<Joint_solution> IK_Calculate_Result();
	CRobot();
private:
	Eigen::MatrixXd RTCP_J6_Matrix(); // from TCP  calculate the Robot J6 flange frame and return the value;
	bool IK_Robot_Calculation_J1(Eigen::MatrixXd RJ_6); // for calculate all the possible J1 Solutions;
	bool IK_Robot_Calculation_J2(Eigen::MatrixXd RJ_6);// for calculate all the possible J1 Solutions;
	bool IK_Robot_Calculation_J3(Eigen::MatrixXd RJ_6);// for calculate all the possible J1 Solutions;
	std::vector<double> IK_Robot_Calculation_J4_J5_J6(double J1, double J2, double J3, Eigen::MatrixXd RJ_6);
	bool confirm_J5_Location(Eigen::Vector3d J5_Location,double J1, double J2, double J3);
	


};

