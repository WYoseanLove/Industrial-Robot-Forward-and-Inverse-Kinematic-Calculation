#include <iostream>
#include "CKUKA.h"
/*
Define the normally used KUKA Robot DH model value, if the model type is not avaliable in the data group below,
please search the data in KUKA Global websit and extend the data below;

*/
double KUKA[][6] = {
{500, 1045, 1300, 55, 1525, 290},// KR240R3330 0
{500, 1045, 1300, 55, 1275, 290},// KR280R3080 1
{500, 1045, 1300, 55, 1525, 290},// KR340R3330 2
{500, 1045, 1300, 55, 1025, 290},// KR360R2830 3
{350, 675, 1150, 41, 1200, 215},// KR210R2700 4
{350, 675, 1150, 41, 1200, 215},// KR90R2700  5
{350, 675, 1350, 41, 1200, 215},// KR90R2900  6
{500, 1045, 1300, 55, 1275, 290},// KR420R3080 7
{500, 1045, 1300, 55, 1525, 290},// KR420R3330 8
{500, 1045, 1300, 55, 1025, 290},// KR500R2830 9
{500, 1045, 1300, 55, 1275, 290}// KR510R3080 10
};


CKUKA::CKUKA()
{

	/*
     define and update the J*_offset value to be same with Robot @ Home posture
    */

	J1_offset = 0;
	J2_offset = -90;
	J3_offset = 90;
	J4_offset = 0;
	J5_offset = 0;
	J6_offset = 0;
}

int CKUKA::Initial(KUKARob robotType)
{
	a1 = KUKA[robotType][0];
	d1 = KUKA[robotType][1];
	a2 = KUKA[robotType][2];
	a3 = KUKA[robotType][3];
	d4 = KUKA[robotType][4];
	d5 = KUKA[robotType][5];
	
	return 0;
}

std::vector<Joint_solution> CKUKA::IK_KUKA_Result(double * TCPX, double * TCPY, double * TCPZ, double * TCPRX, double * TCPRY, double * TCPRZ)
{
	std::vector<Joint_solution> results_kuka;
	//std::vector<double*> final_results_kuka;
	results_kuka = IK_Calculate_Result();
	if (results_kuka.empty())
		std::cout << "No KUKA Robot IK Solution calculated !!!" << std::endl;
	else
	{

		for (int i = 0; i < results_kuka.size(); i++)
		{

			double solution[6];
			solution[0] = results_kuka[i].j1;
			solution[1] = results_kuka[i].j2;
			solution[2] = results_kuka[i].j3;
			solution[3] = results_kuka[i].j4;
			solution[4] = results_kuka[i].j5;
			solution[5] = results_kuka[i].j6;

			if ((Robot_Joint_Value_Check(solution[1], -130, 20) < -130) || (Robot_Joint_Value_Check(solution[1], -130, 20) > 20))
			{
				results_kuka.erase(results_kuka.begin() + i);
				i--;
				continue;

			}
			else
			{
				//std::cout << "below is the " << i << " calculation solutions" << std::endl;
				solution[0] = Robot_Joint_Value_Check(solution[0], -185, 185);
				solution[1] = Robot_Joint_Value_Check(solution[1], -130, 20);
				solution[2] = Robot_Joint_Value_Check(solution[2], -100, 144);
				solution[3] = Robot_Joint_Value_Check(solution[3], -350, 350);
				solution[4] = Robot_Joint_Value_Check(solution[4], -120, 120);
				solution[5] = Robot_Joint_Value_Check(solution[5], -350, 350);
				if (check_robot_solution(solution))
				{
					 results_kuka[i].j1=solution[0];
					 results_kuka[i].j2 = solution[1];
					 results_kuka[i].j3 = solution[2];
					 results_kuka[i].j4 = solution[3];
					 results_kuka[i].j5 = solution[4];
					 results_kuka[i].j6 = solution[5];

				}
				else
				{
					results_kuka.erase(results_kuka.begin()+i);
					i--;
					continue;
				}
					
				



			}





		}





	}

	return results_kuka;
}

int CKUKA::KW_KUKA_Result(double * TCPX, double * TCPY, double * TCPZ, double * TCPRX, double * TCPRY, double * TCPRZ)
{
	
	Eigen::MatrixXd RT = KW_Robot(&a1, &d1, &a2, &a3, &d4, &d5, TCPX, TCPY, TCPZ, TCPRX, TCPRY, TCPRZ);

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
	return 0;
}
