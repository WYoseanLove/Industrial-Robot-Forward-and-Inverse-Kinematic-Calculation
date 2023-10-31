#pragma once
#include "CRobot.h"
enum FANUCRob {
	R_2000IC_165F,
	R_2000IC_210F,
	R_2000IC_210L,
	R_2000IC_270F,
	R_2000IC_165R,
	M_900IB_400L,

};
class CFANUC :
	public CRobot
{
public:
	CFANUC();
	int Initial(FANUCRob robotType);
	std::vector<Joint_solution> IK_FANUC_Result(double* TCPX, double* TCPY, double* TCPZ, double* TCPRX, double* TCPRY, double* TCPRZ, FANUCRob robotType);
	Eigen::VectorXd KW_FANUC_Result(double * TCPX, double * TCPY, double * TCPZ, double * TCPRX, double * TCPRY, double * TCPRZ, FANUCRob robotType);
	Eigen::MatrixXd KW_Robot(double* a1, double* d1, double* a2, double* a3, double* d4, double* d5, double* TCPX, double* TCPY, double* TCPZ, double* TCPRX, double* TCPRY, double* TCPRZ);
	
};

