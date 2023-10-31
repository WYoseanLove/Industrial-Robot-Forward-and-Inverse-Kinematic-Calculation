#include <iostream>
#include "CABB.h"
#include "CRobot.h"

double ABB[][6] = {
{410, 780, 1075, -165, 806, 250},// IRB7600_230 0
{410, 780, 1075, -165, 1056, 250},// IRB7600_255 1
{410, 780, 1075, -165, 1306, 250},// IRB7600_280 2
{410, 780, 1075, -165, 1556, 250},// IRB7600_310 3
{410, 780, 1075, -165, 2012, 250},// IRB7600_350 4
{320,780,1135,-200,1182.5,200}, //IRB6700_265 5
{320,780,1280,-200,1182.5,200}, //IRB6700_280 6
{320,780,1135,-200,1592.5,200}, //IRB6700_305 7
{320,780,1280,-200,1592.5,200}, //IRB6700_320 8
{320,780,1125,-200,1142.5,200}, //IRB6700_260 9
{320,780,1125,-200,1393,200}, //IRB6700_285 10
{350,780,1145,-200,1212.5,220}, //IRB6700_270 11
{350,780,1145,-200,1462.5,220}, //IRB6700_300 12
{600,630,1280,-200,1142,200}, //IRB6650S_300 13
{600,630,1280,-200,1592,200}, //IRB6650S_350 14
{600,630,1280,-200,2042,200}, //IRB6650S_390 15
};




/*
ABB robot kinematic calculation is done according to kuka robot calculation;

J1 , J4 and J6 rotation angle is opposite with KUKA J1, J4 and J6, other axis is same rotation angle with KUKA robot

for all ABB robot home posture the J1-J6 is 0 deg , so all the J*_offset value will be zero which will be automatic remove it from the calculation

*/

CABB::CABB()
{
	// from ABB robot kinematix know when ABB robot is at HOME posture, all the J1-J6 is at zero degree,
	// so give the J1_offset to J6_offset value as 0 deg;
	J1_offset = 0;
	J2_offset = 0;
	J3_offset = 0;
	J4_offset = 0;
	J5_offset = 0;
	J6_offset = 0;
	
}

int CABB::Initial(ABBRob robotType)
{
	a1 = ABB[robotType][0];
	d1 = ABB[robotType][1];
	a2 = ABB[robotType][2];
	a3 = ABB[robotType][3];
	d4 = ABB[robotType][4];
	d5 = ABB[robotType][5];

	
	return 0;
}

std::vector<Joint_solution> CABB::IK_ABB_Result(double* TCPX, double* TCPY, double* TCPZ, double* TCPRX, double* TCPRY, double* TCPRZ)
{
	std::vector<Joint_solution> results_abb;
	 results_abb= IK_Calculate_Result();
	 
	 if (results_abb.empty())
		 std::cout << "No ABB Robot IK Solution calculated !!!" << std::endl;
	 else
	 {
		 
		 for (int i = 0; i < results_abb.size(); i++)
		 {

			 double solution[6];
			 solution[0] = results_abb[i].j1;
			 solution[1] = results_abb[i].j2;
			 solution[2] = results_abb[i].j3;
			 solution[3] = results_abb[i].j4;
			 solution[4] = results_abb[i].j5;
			 solution[5] = results_abb[i].j6;
			 if ((Robot_Joint_Value_Check(solution[1], -60, 85) < -60) || (Robot_Joint_Value_Check(solution[1], -60, 85) > 85))
			 {
				 results_abb.erase(results_abb.begin() + i);
				 i--;
				 continue;

			 }
			 else
			 {
				 solution[0] = -solution[0];
				 solution[3] = -solution[3];
				 solution[5] = -solution[5];
				 //std::cout << "below is the " << i << " calculation solutions" << std::endl;
				 solution[0] = Robot_Joint_Value_Check(solution[0], -180, 180);
				 solution[1] = Robot_Joint_Value_Check(solution[1], -60, 85) ;
				 solution[2] = Robot_Joint_Value_Check(solution[2], -180, 60);
				 solution[3] = Robot_Joint_Value_Check(solution[3], -300, 300);
				 solution[4] = Robot_Joint_Value_Check(solution[4], -100, 100);
				 solution[5] = Robot_Joint_Value_Check(solution[5], -360, 360);

				 if (check_robot_solution(solution))
				 {
					 results_abb[i].j1 = solution[0];
					 results_abb[i].j2 = solution[1];
					 results_abb[i].j3 = solution[2];
					 results_abb[i].j4 = solution[3];
					 results_abb[i].j5 = solution[4];
					 results_abb[i].j6 = solution[5];

				 }
				 else
				 {
					 results_abb.erase(results_abb.begin() + i);
					 i--;
					 continue;
				 }
					 

				
			


			 }





		 }





	 }

	 return results_abb;
}

Eigen::VectorXd CABB::KW_ABB_Result(double * TCPX, double * TCPY, double * TCPZ, double * TCPRX, double * TCPRY, double * TCPRZ)
{
	/*
	From Robot Simulation knows, ABB Robot J1, J4, J6 Rotation deg is opposite with KUKA robot, so ABB construction fuction should be opposite for below Joint value;

	*/
	J1 = -J1;
	J4 = -J4;
	J6 = -J6;

	Eigen::MatrixXd RT=KW_Robot(&a1, &d1, &a2, &a3, &d4, &d5, TCPX, TCPY, TCPZ,TCPRX,  TCPRY, TCPRZ);

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
	std::cout << Tx << "  " << Ty << "  " << Tz << "  " << rx << "  " << ry << "  " << rz << "  " << std::endl;
	Eigen::VectorXd KW_Result(6);
	KW_Result << Tx, Ty, Tz, rx, ry, rz;
	return KW_Result;
}


