#pragma once
#include "CRobot.h"
enum ABBRob {
	IRB7600_230,
	IRB7600_255,
	IRB7600_280,
	IRB7600_310,
	IRB7600_350,
    IRB6700_265,
    IRB6700_280,
    IRB6700_305,
	IRB6700_320,
	IRB6700_260,
	IRB6700_285,
	IRB6700_270,
	IRB6700_300,
	IRB6650S_300,
	IRB6650S_350,
	IRB6650S_390,

};
class CABB :
	public CRobot
{
public:
	CABB();
	int Initial(ABBRob robotType);
	std::vector<Joint_solution> IK_ABB_Result(double* TCPX, double* TCPY, double* TCPZ, double* TCPRX, double* TCPRY, double* TCPRZ);
	Eigen::VectorXd KW_ABB_Result(double * TCPX, double * TCPY, double * TCPZ, double * TCPRX, double * TCPRY, double * TCPRZ);
};

