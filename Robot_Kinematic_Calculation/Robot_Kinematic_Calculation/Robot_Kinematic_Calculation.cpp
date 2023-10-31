// Robot_Kinematic_Calculation.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "CRobot.h"
#include "CKUKA.h"
#include "CABB.h"
#include "CFANUC.h"
#include "CKAWASAKI.h"
extern double KUKA[][6];
extern double ABB[][6];
extern double FANUC[][6];
extern double KAWASAKI[][6];
int main()
{
    std::cout << "Hello World!\n";
	
	CKUKA kuka;
	kuka.Initial(KR340R3330);

	kuka.TCPX = -23.39;
	kuka.TCPY = 87.31;
	kuka.TCPZ = 1553;
	kuka.TCPRX = -75;
	kuka.TCPRY = -90;
	kuka.TCPRZ = 0;// FROM Flange Frame To TCP frame;
	

	kuka.J1 = 38.25;
	kuka.J2 = -25.55;
	kuka.J3 = -43.64;
	kuka.J4 = 279.97;
	kuka.J5 = 51.11;
	kuka.J6 = -56.98;


	std::cout << "Below is an example fpr KUKA Robot" << std::endl;

	kuka.KW_KUKA_Result(&kuka.TCPX, &kuka.TCPY, &kuka.TCPZ, &kuka.TCPRX, &kuka.TCPRY, &kuka.TCPRZ);

	kuka.TCPX = -1553;
	kuka.TCPY = 0;
	kuka.TCPZ = -90.39;
	kuka.TCPRX = 90;
	kuka.TCPRY = 15;
	kuka.TCPRZ = 90; // FROM TCP To Flange frame;


	kuka.rx = 12.08;
	kuka.ry = 33.95;
	kuka.rz = 35.42;

	kuka.Tx = 3917.58;
	kuka.Ty = 899.18;
	kuka.Tz = 387.04;

	std::vector<Joint_solution> kuka_solution = kuka.IK_KUKA_Result(&kuka.TCPX, &kuka.TCPY, &kuka.TCPZ, &kuka.TCPRX, &kuka.TCPRY, &kuka.TCPRZ);

	if (kuka_solution.size() != 0)
	{
		std::cout << "below is all the kuka IK solutions:" << std::endl;
		for (int i = 0; i < kuka_solution.size(); i++)
		{
			std::cout << "The " << i << "st solution" << std::endl;

			std::cout << kuka_solution[i].j1 << std::endl;
			std::cout << kuka_solution[i].j2 << std::endl;
			std::cout << kuka_solution[i].j3 << std::endl;
			std::cout << kuka_solution[i].j4 << std::endl;
			std::cout << kuka_solution[i].j5 << std::endl;
			std::cout << kuka_solution[i].j6 << std::endl;
		}


	}
	else
		std::cout << "No solution calculated" << std::endl;
	
	CABB abb;
	abb.Initial(IRB7600_255);
	// FROM Flange Frame To TCP frame; 
	// Data got from process simulation, different Tool exist different TCP value, data must update below when Tool change for Robot
	abb.TCPX = -23.39;
	abb.TCPY = 87.31;
	abb.TCPZ = 1553;
	abb.TCPRX = -75;
	abb.TCPRY = -90;
	abb.TCPRZ = 0;
	//

	abb.J1 = 38.25;
	abb.J2 = -25.55;
	abb.J3 = -43.64;
	abb.J4 = 279.97;
	abb.J5 = 51.11;
	abb.J6 = -56.98;

	std::cout << "Below is an example fpr ABB Robot" << std::endl;
	abb.KW_ABB_Result(&abb.TCPX, &abb.TCPY, &abb.TCPZ, &abb.TCPRX, &abb.TCPRY, &abb.TCPRZ);
	abb.TCPX = -1553;
	abb.TCPY = 0;
	abb.TCPZ = -90.39;
	abb.TCPRX = 90;
	abb.TCPRY = 15;
	abb.TCPRZ = 90; // FROM TCP To Flange frame;


	abb.rx = -21.31;
	abb.ry = 26.29;
	abb.rz = 47.44;

	abb.Tx = 2513.1;
	abb.Ty = 2784.72;
	abb.Tz = 1367.53;

	std::vector<Joint_solution> abb_solution = abb.IK_ABB_Result(&abb.TCPX, &abb.TCPY, &abb.TCPZ, &abb.TCPRX, &abb.TCPRY, &abb.TCPRZ);
	if (abb_solution.size() != 0)
	{
		std::cout << "below is all the abb IK solutions:" << std::endl;
		for (int i = 0; i < abb_solution.size(); i++)
		{
			std::cout << "The " << i << "st solution" << std::endl;

			std::cout << abb_solution[i].j1 << std::endl;
			std::cout << abb_solution[i].j2 << std::endl;
			std::cout << abb_solution[i].j3 << std::endl;
			std::cout << abb_solution[i].j4 << std::endl;
			std::cout << abb_solution[i].j5 << std::endl;
			std::cout << abb_solution[i].j6 << std::endl;
		}


	}
	else
		std::cout << "No solution calculated" << std::endl;
	
	
	CFANUC fanuc;
	fanuc.Initial(M_900IB_400L);
	// FROM Flange Frame To TCP frame; 
	// Data got from process simulation, different Tool exist different TCP value, data must update below when Tool change for Robot
	fanuc.TCPX = -23.39;
	fanuc.TCPY = 87.31;
	fanuc.TCPZ = 1553;
	fanuc.TCPRX = -75;
	fanuc.TCPRY = -90;
	fanuc.TCPRZ = 0;
	//

	fanuc.J1 =27.19;
	fanuc.J2 = -29.34;
	fanuc.J3 = 24.05;
	fanuc.J4 =-13.16;
	fanuc.J5 =-44.13;
	fanuc.J6 = -25.75;


	std::cout << "Below is an example fpr fanuc Robot" << std::endl;
	fanuc.KW_FANUC_Result(&fanuc.TCPX, &fanuc.TCPY, &fanuc.TCPZ, &fanuc.TCPRX, &fanuc.TCPRY, &fanuc.TCPRZ, M_900IB_400L);
	fanuc.TCPX = -1553;
	fanuc.TCPY = 0;
	fanuc.TCPZ = -90.39;
	fanuc.TCPRX = 90;
	fanuc.TCPRY = 15;
	fanuc.TCPRZ = 90; // FROM TCP To Flange frame;


	fanuc.rx =-148.51;
	fanuc.ry = 34.47;
	fanuc.rz =-44.24;

	fanuc.Tx = 2076.18;
	fanuc.Ty = -2787.95;
	fanuc.Tz = 523.94;

	std::vector<Joint_solution> fanuc_solution = fanuc.IK_FANUC_Result(&fanuc.TCPX, &fanuc.TCPY, &fanuc.TCPZ, &fanuc.TCPRX, &fanuc.TCPRY, &fanuc.TCPRZ, M_900IB_400L);
	if (fanuc_solution.size() != 0)
	{
		std::cout << "below is all the fanuc IK solutions:" << std::endl;
		for (int i = 0; i < fanuc_solution.size(); i++)
		{
			std::cout << "The " << i << "st solution" << std::endl;

			std::cout << fanuc_solution[i].j1 << std::endl;
			std::cout << fanuc_solution[i].j2 << std::endl;
			std::cout << fanuc_solution[i].j3 << std::endl;
			std::cout << fanuc_solution[i].j4 << std::endl;
			std::cout << fanuc_solution[i].j5 << std::endl;
			std::cout << fanuc_solution[i].j6 << std::endl;
		}


	}
	else
		std::cout << "No solution calculated" << std::endl;


    CKAWASAKI kawasaki;
    kawasaki.Initial(BX200L);
	kawasaki.TCPX = -23.39;
	kawasaki.TCPY = 87.31;
	kawasaki.TCPZ = 1553;
	kawasaki.TCPRX = -75;
	kawasaki.TCPRY = -90;
	kawasaki.TCPRZ = 0;
	//

	kawasaki.J1 =50;
	kawasaki.J2 = 60;
	kawasaki.J3 =-30;
	kawasaki.J4 =30;
	kawasaki.J5 =20;
	kawasaki.J6 =30;

	std::cout << "Below is an example fpr KAWASAKI Robot" << std::endl;
	kawasaki.KW_KAWASAKI_Result(&kawasaki.TCPX, &kawasaki.TCPY, &kawasaki.TCPZ, &kawasaki.TCPRX, &kawasaki.TCPRY, &kawasaki.TCPRZ, BX200L);
	kawasaki.TCPX = -1553;
	kawasaki.TCPY = 0;
	kawasaki.TCPZ = -90.39;
	kawasaki.TCPRX = 90;
	kawasaki.TCPRY = 15;
	kawasaki.TCPRZ = 90; // FROM TCP To Flange frame;


	kawasaki.rx = 57.71;
	kawasaki.ry =22.3;
	kawasaki.rz =15.4;

	kawasaki.Tx = 2303.87;
	kawasaki.Ty = 1241.89;
	kawasaki.Tz = 582.54;

	std::vector<Joint_solution> kawasaki_solution=kawasaki.IK_KAWASAKI_Result(&kawasaki.TCPX, &kawasaki.TCPY, &kawasaki.TCPZ, &kawasaki.TCPRX, &kawasaki.TCPRY, &kawasaki.TCPRZ, BX200L);

	if (kawasaki_solution.size() != 0)
	{
		std::cout << "below is all the kawasaki IK solutions:" << std::endl;
		for (int i = 0; i < kawasaki_solution.size(); i++)
		{
			std::cout << "The " << i<< "st solution"<< std::endl;

			std::cout << kawasaki_solution [i].j1<< std::endl;
			std::cout << kawasaki_solution[i].j2 << std::endl;
			std::cout << kawasaki_solution[i].j3 << std::endl;
			std::cout << kawasaki_solution[i].j4 << std::endl;
			std::cout << kawasaki_solution[i].j5 << std::endl;
			std::cout << kawasaki_solution[i].j6 << std::endl;
		}


	}
	else
		std::cout << "No solution calculated" << std::endl;


}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
