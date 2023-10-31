#include <iostream>
#include "CFANUC.h"
double FANUC[][6] = {
{312, 670, 1075, -225, 1280, 215},// R_2000IC_165F
{312, 670, 1075, -225, 1280, 215},// R_2000IC_210F
{312, 670, 1075, -225, 1730, 240},// R_2000IC_210L
{312, 670, 1075, -225, 1280, 240},// R_2000IC_270F
{720,600,1075,-225,1280,215}, //R_2000IC_165R
{410,940,1120,-250,2180,300},//M_900IB_400L
};

/*
FANUC robot kinematic calculation is done according to kuka robot calculation;

J1 , J3 and J5 rotation angle is opposite with KUKA J1, J4 and J6, other axis is same rotation angle with KUKA robot

for all FANUC robot home posture the J1-J6 is 0 deg , so all the J*_offset value will be zero which will be automatic remove it from the calculation

*/
CFANUC::CFANUC()
{
	// from FANUC robot kinematix know when FANUC robot is at HOME posture, all the J1-J6 is at zero degree,
	// so give the J1_offset to J6_offset value as 0 deg;
	J1_offset = 0;
	J2_offset = 0;
	J3_offset = 0;
	J4_offset = 0;
	J5_offset = 0;
	J6_offset = 0;

}

int CFANUC::Initial(FANUCRob robotType)
{
	a1 = FANUC[robotType][0];
	d1 = FANUC[robotType][1];
	a2 = FANUC[robotType][2];
	a3 = FANUC[robotType][3];
	d4 = FANUC[robotType][4];
	d5 = FANUC[robotType][5];

	if (robotType == R_2000IC_165R)
	{
		J2_offset = -90;
		J3_offset = 90;
	}




	return 0;
}

std::vector<Joint_solution> CFANUC::IK_FANUC_Result(double * TCPX, double * TCPY, double * TCPZ, double * TCPRX, double * TCPRY, double * TCPRZ, FANUCRob robotType)
{
	std::vector<Joint_solution> results_FANUC;
	results_FANUC = IK_Calculate_Result();

	if (results_FANUC.empty())
		std::cout << "No FANUC Robot IK Solution calculated !!!" << std::endl;
	else
	{

		for (int i = 0; i < results_FANUC.size(); i++)
		{

			double solution[6];
			solution[0] = results_FANUC[i].j1;
			solution[1] = results_FANUC[i].j2;
			solution[2] = results_FANUC[i].j3;
			solution[3] = results_FANUC[i].j4;
			solution[4] = results_FANUC[i].j5;
			solution[5] = results_FANUC[i].j6;

			if ((Robot_Joint_Value_Check(solution[1], -60, 76) < -60) || (Robot_Joint_Value_Check(solution[1], -60, 76) > 76))
			{
				results_FANUC.erase(results_FANUC.begin() + i);
				i--;
				continue;

			}
			else
			{
				solution[0] = -solution[0];
				solution[2] = -solution[2];
				solution[2] -= solution[1];
				solution[2] += J3_offset;
				solution[4] = -solution[4];
				solution[5] += 180;

				solution[0] = Robot_Joint_Value_Check(solution[0], -185, 185);

				if (robotType== R_2000IC_165R)
				{
					solution[1] = Robot_Joint_Value_Check(solution[1], -79, 80);
					solution[2] = Robot_Joint_Value_Check(solution[2], -70.8 - solution[1], 180 - solution[1]);
				}
				else if(robotType == M_900IB_400L)
				{
					solution[1] = Robot_Joint_Value_Check(solution[1], -64, 70);
					solution[2] = Robot_Joint_Value_Check(solution[2], -70 - solution[1], 30);

				}
				else
				{
					solution[1] = Robot_Joint_Value_Check(solution[1], -60, 76);
					solution[2] = Robot_Joint_Value_Check(solution[2], -79.13 - solution[1], 180 - solution[1]);

				}
				solution[3]= Robot_Joint_Value_Check(solution[3], -360, 360);
				solution[4]= Robot_Joint_Value_Check(solution[4], -125, 125);
				solution[5]= Robot_Joint_Value_Check(solution[5], -360, 360);

				
				if (check_robot_solution(solution))
				{
					results_FANUC[i].j1 = solution[0];
					results_FANUC[i].j2 = solution[1];
					results_FANUC[i].j3 = solution[2];
					results_FANUC[i].j4 = solution[3];
					results_FANUC[i].j5 = solution[4];
					results_FANUC[i].j6 = solution[5];

				}
				else
				{
					results_FANUC.erase(results_FANUC.begin() + i);
					i--;
					continue;
				}
				



			}





		}





	}

	return results_FANUC;
}

Eigen::VectorXd CFANUC::KW_FANUC_Result(double * TCPX, double * TCPY, double * TCPZ, double * TCPRX, double * TCPRY, double * TCPRZ, FANUCRob robotType)
{
	/*
	From Robot Simulation knows, ABB Robot J1, J4, J6 Rotation deg is opposite with KUKA robot, so ABB construction fuction should be opposite for below Joint value;

	*/
	J1 = -J1;
	J3 = -J3;
	J5 = -J5;

	if (robotType== R_2000IC_165R)
	{
		
		J3 =2*J3_offset+J3;

	}
	Eigen::MatrixXd RT = CFANUC::KW_Robot(&a1, &d1, &a2, &a3, &d4, &d5, TCPX, TCPY, TCPZ, TCPRX, TCPRY, TCPRZ);

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

Eigen::MatrixXd CFANUC::KW_Robot(double * a1, double * d1, double * a2, double * a3, double * d4, double * d5, double * TCPX, double * TCPY, double * TCPZ, double * TCPRX, double * TCPRY, double * TCPRZ)
{
	Eigen::MatrixXd RT;
	RT = Eigen::MatrixXd(4, 4);
	RT = KW_Robot_Joint(*a1, 0, *d1, 0, -90, -90, (J1 - J1_offset));
	RT *= KW_Robot_Joint(*a2, 0, 0, 0, 0, 0, -(J2 - J2_offset));
	RT *= KW_Robot_Joint(-*a3, *d4, 0, -90, 0, 0, -(J3 - J3_offset)+ (J2 - J2_offset));
	RT *= KW_Robot_Joint(0, 0, 0, 90, 0, 0, (J4 - J4_offset));
	/*
	from the J4 frame to calculate the J5 Joint location and rotation frame
		from the J4 frame to J5 frame will be through below steps :
	1. J3 frame should be move 0 along with X aix direction;
	2. J3 frame should be move *d5 along with Y aix direction;
	3. J3 frame should be move 0 along with Z aix direction;
	4. J3 frame should be rotate 0 deg  with Z aix;
	5. J3 frame should be rotate 0 deg with Y aix;  HERE is special for FANUC robot
	6. J3 frame should be rotate - 90 deg with X aix;
	after 6 steps, the base frame will be at J1 location and same direction with J1;
	*/
	RT *= KW_Robot_Joint(0, *d5, 0, -90, 0, 0, -(J5 - J5_offset));
	RT *= KW_Robot_Joint(*TCPX, *TCPY, *TCPZ, *TCPRX, *TCPRY, *TCPRZ, (J6 - J6_offset));

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
