#include <iostream>
#include "CKAWASAKI.h"
#include "CRobot.h"
double KAWASAKI[][6] = {
{200, 530, 1160, -230, 1250, 225},// BX200L 0
{325,1160,1100, -250,1600,310}, //MX350L 1
};

CKAWASAKI::CKAWASAKI()
{
	// from KAWASAKI robot kinematix know when FANUC robot is at HOME posture, all the J1-J6 is at zero degree,
	// so give the J1_offset to J6_offset value as 0 deg;
	J1_offset =0;
	J2_offset = 0;
	J3_offset = 0;
	J4_offset = 0;
	J5_offset = 0;
	J6_offset = 0;
}

int CKAWASAKI::Initial(KAWASAKIRob robotType)
{
	//CABB::J1-J6 data input can be directly set to KAWASAKI input, all the transform between ABB and kawasaki will be done here;
	a1 = KAWASAKI[robotType][0];
	d1 = KAWASAKI[robotType][1];
	a2 = KAWASAKI[robotType][2];
	a3 = KAWASAKI[robotType][3];
	d4 = KAWASAKI[robotType][4];
	d5 = KAWASAKI[robotType][5];


	return 0;
}

Eigen::VectorXd CKAWASAKI::KW_KAWASAKI_Result(double * TCPX, double * TCPY, double * TCPZ, double * TCPRX, double * TCPRY, double * TCPRZ, KAWASAKIRob robotType)
{
	J1 =  J1;
	if (robotType == MX350L)
	{
		
		J3 += J2;
	}
	J3 = -J3;
	J4 = -J4;
	J5 = -J5;
	J6 = -(J6-90);

	Eigen::MatrixXd RT=KW_Robot(&a1, &d1, &a2, &a3, &d4, &d5, TCPX, TCPY, TCPZ, TCPRX, TCPRY, TCPRZ);
	Eigen::MatrixXd Rt(4, 4);
	Rt << cos(M_PI / 2), -sin(M_PI / 2), 0, 0,
		sin(M_PI / 2), cos(M_PI / 2), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 0;

	RT = Rt * RT;
	
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

std::vector<Joint_solution> CKAWASAKI::IK_KAWASAKI_Result(double * TCPX, double * TCPY, double * TCPZ, double * TCPRX, double * TCPRY, double * TCPRZ, KAWASAKIRob robotType)
{
	std::vector<Joint_solution> results_KAWASAKI;
	
	results_KAWASAKI = IK_Calculate_Result();
	if (results_KAWASAKI.empty())
		std::cout << "No KAWASAKI Robot IK Solution calculated !!!" << std::endl;
	else
	{
		
		for (int i = 0; i < results_KAWASAKI.size(); i++)
		{

			double solution[6];
			solution[0] = results_KAWASAKI[i].j1;
			solution[1] = results_KAWASAKI[i].j2;
			solution[2] = results_KAWASAKI[i].j3;
			solution[3] = results_KAWASAKI[i].j4;
			solution[4] = results_KAWASAKI[i].j5;
			solution[5] = results_KAWASAKI[i].j6;
			if (robotType == BX200L)
			{
				if ((Robot_Joint_Value_Check(solution[1], -60, 76) < -60) || (Robot_Joint_Value_Check(solution[1], -60, 76) > 76))
				{
					results_KAWASAKI.erase(results_KAWASAKI.begin() + i);
					i--;
					continue;

				}
				else
				{
					solution[0] = solution[0] + 90;
					solution[2] = -solution[2];
					solution[3] = -solution[3];
					solution[4] = -solution[4];
					solution[5] = 90 - solution[5];
					//std::cout << "below is the " << i << " calculation solutions" << std::endl;
					solution[0]= Robot_Joint_Value_Check(solution[0], -160, 160);
					
					solution[1] = Robot_Joint_Value_Check(solution[1], -60, 76);
					if (solution[1] > 0)
						solution[2] = Robot_Joint_Value_Check(solution[2], -75, 90) ;
					else
					{
						solution[1] = Robot_Joint_Value_Check(solution[1], -60, 76);
						solution[2] = Robot_Joint_Value_Check(solution[2], -75, 90 + solution[1]) ;
					}
						

					solution[3] = Robot_Joint_Value_Check(solution[3], -210, 210);
					solution[4] = Robot_Joint_Value_Check(solution[4], -125, 125);
					solution[5] = Robot_Joint_Value_Check(solution[5], -210, 210);



					if (check_robot_solution(solution))
					{
						results_KAWASAKI[i].j1 = solution[0];
						results_KAWASAKI[i].j2 = solution[1];
						results_KAWASAKI[i].j3 = solution[2];
						results_KAWASAKI[i].j4 = solution[3];
						results_KAWASAKI[i].j5 = solution[4];
						results_KAWASAKI[i].j6 = solution[5];

					}
					else
					{
						results_KAWASAKI.erase(results_KAWASAKI.begin() + i);
						i--;
						continue;
					}


					



				}



			}
			if (robotType == MX350L)
			{
				if ((Robot_Joint_Value_Check(solution[1], -45, 65) < -45) || (Robot_Joint_Value_Check(solution[1], -45,65) > 65))
				{
					results_KAWASAKI.erase(results_KAWASAKI.begin() + i);
					i--;
					continue;

				}
				else
				{
					solution[0] = solution[0] + 90;
					solution[2] = -solution[2];
					solution[2] -= solution[1];
					solution[3] = -solution[3];
					solution[4] = -solution[4];
					solution[5] = 90 - solution[5];
					
					solution[0] =Robot_Joint_Value_Check(solution[0], -180, 180);
					solution[1] = Robot_Joint_Value_Check(solution[1], -45, 65);
					solution[2] = Robot_Joint_Value_Check(solution[2], -55 - solution[1],20);

					
					solution[3] = Robot_Joint_Value_Check(solution[3], -360, 360);
					solution[4] = Robot_Joint_Value_Check(solution[4], -110, 110);
					solution[5] = Robot_Joint_Value_Check(solution[5], -360, 360);


					if (check_robot_solution(solution))
					{
						results_KAWASAKI[i].j1 = solution[0];
						results_KAWASAKI[i].j2 = solution[1];
						results_KAWASAKI[i].j3 = solution[2];
						results_KAWASAKI[i].j4 = solution[3];
						results_KAWASAKI[i].j5 = solution[4];
						results_KAWASAKI[i].j6 = solution[5];

					}
					else
					{
						results_KAWASAKI.erase(results_KAWASAKI.begin() + i);
						i--;
						continue;
					}


				}



			}



		}





	}

	return results_KAWASAKI;
}

