#pragma once
#include "CRobot.h"

enum KAWASAKIRob {
	BX200L,
	MX350L,

};
class CKAWASAKI:
	public CRobot
{
public:
	CKAWASAKI();
	int Initial(KAWASAKIRob robotType);
	Eigen::VectorXd KW_KAWASAKI_Result(double * TCPX, double * TCPY, double * TCPZ, double * TCPRX, double * TCPRY, double * TCPRZ, KAWASAKIRob robotType);
	std::vector<Joint_solution> IK_KAWASAKI_Result(double* TCPX, double* TCPY, double* TCPZ, double* TCPRX, double* TCPRY, double* TCPRZ, KAWASAKIRob robotType);
};

