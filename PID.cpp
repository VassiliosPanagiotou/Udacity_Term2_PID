#include "PID.h"

using namespace std;

PID::PID(double Kp, double Ki, double Kd, bool runTwiddle)
	: m_p_error(0.0)
	, m_i_error(0.0)
	, m_d_error(0.0)
	, m_Kp(Kp)
	, m_Ki(Ki)
	, m_Kd(Kd)
	, m_RunTwiddle(runTwiddle){
	//
}


double PID::getSteerValue(double cte)
{
	m_d_error = cte - m_p_error;
	m_p_error = cte;
	m_i_error += cte;

	if (m_RunTwiddle)
	{

	}

	// return steering value
	return -(m_Kp * m_p_error + m_Ki * m_i_error + m_Kd * m_d_error);
	// return -(m_Kp * m_p_error);
	// return -(m_Kp * m_p_error + m_Ki * m_i_error);
	// return -(m_Kp * m_p_error + m_Kd * m_d_error);
}

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	m_p_error = 0.0;
	m_i_error = 0.0;
	m_d_error = 0.0;

	m_Kp = Kp;
	m_Ki = Ki;
	m_Kd = Kd;
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
	return 0.0;
}

