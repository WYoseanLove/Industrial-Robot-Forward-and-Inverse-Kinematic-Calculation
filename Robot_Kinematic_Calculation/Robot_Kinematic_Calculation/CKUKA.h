#pragma once
#include "CRobot.h"

enum KUKARob {
	KR240R3330, 
	KR280R3080, 
	KR340R3330, 
	KR360R2830, 
	KR210R2700,
	KR90R2700,
	KR90R2900,
	KR420R3080,
	KR420R3330,
	KR500R2830,
	KR510R3080,
};
class CKUKA :
	public CRobot
{
public: 
	CKUKA();
	int Initial(KUKARob robotType);
	std::vector<Joint_solution> IK_KUKA_Result(double* TCPX, double* TCPY, double* TCPZ, double* TCPRX, double* TCPRY, double* TCPRZ);
	int KW_KUKA_Result(double * TCPX, double * TCPY, double * TCPZ, double * TCPRX, double * TCPRY, double * TCPRZ);

	
};

