#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;

PID::PID(double Kp, double Ki, double Kd, bool runTwiddle)
	: m_p_error(0.0)
	, m_i_error(0.0)
	, m_d_error(0.0)
	, m_Kp(Kp)
	, m_Ki(Ki)
	, m_Kd(Kd)
	, m_RunTwiddle(runTwiddle)
	, m_StepCount(0)
{
	// Init twiddle struct
	m_twiddle.k[0] = Kp;
	m_twiddle.k[1] = Ki;
	m_twiddle.k[2] = Kd;
	m_twiddle.dk[0] = fabs(Kp) / 10.0;
	m_twiddle.dk[1] = fabs(Ki) / 10.0;
	m_twiddle.dk[2] = fabs(Kd) / 10.0;
	m_twiddle.index = 0;
	m_twiddle.isIncrease = true;
	m_twiddle.threshold = 0.01;
	m_twiddle.isInitial = true;
	m_twiddle.bestError = 100000000.0;
}

double PID::getSteerValue(double cte)
{
	++m_StepCount;
	m_d_error = cte - m_p_error;
	m_p_error = cte;
	m_i_error += cte;

	if (m_RunTwiddle && (m_StepCount % 500 == 0) )
	{
		double error = m_i_error / (double) m_StepCount;
		RunTwiddle(error);
		InitError();
	}

	// return steering value
	return -(m_Kp * m_p_error + m_Ki * m_i_error + m_Kd * m_d_error);
	// return -(m_Kp * m_p_error);
	// return -(m_Kp * m_p_error + m_Ki * m_i_error);
	// return -(m_Kp * m_p_error + m_Kd * m_d_error);
}

PID::PID() {}

PID::~PID() {}

void PID::InitError() {
	m_p_error = 0.0;
	m_i_error = 0.0;
	m_d_error = 0.0;
}

void PID::RunTwiddle(double error)
{
	if (fabs(m_twiddle.dk[0]) + fabs(m_twiddle.dk[1]) + fabs(m_twiddle.dk[2]) > m_twiddle.threshold)
	{
		if (m_twiddle.isInitial)
		{
			// Init error
			m_twiddle.bestError = error;
			m_twiddle.isInitial = false;
		}
		else
		{
			// Switch to next step
			if (error > m_twiddle.bestError)
			{
				// Take back current gain and go to next
				if (m_twiddle.isIncrease)
				{
					m_twiddle.k[m_twiddle.index] -= m_twiddle.dk[m_twiddle.index];
					// Twiddle next tries to decrease the same gain
					m_twiddle.isIncrease = false;
				}
				else
				{
					m_twiddle.k[m_twiddle.index] += m_twiddle.dk[m_twiddle.index];
					m_twiddle.dk[m_twiddle.index] *= 0.8;

					m_twiddle.index++;
					m_twiddle.isIncrease = true;
				}
			}
			else
			{
				// Take current gain change and switch to next
				m_twiddle.bestError = error;
				if (m_twiddle.isIncrease)
				{
					m_twiddle.dk[m_twiddle.index] *= 1.2;
					m_twiddle.index++;
				}
				else
				{
					m_twiddle.dk[m_twiddle.index] *= 1.2;
					m_twiddle.index++;
					m_twiddle.isIncrease = true;
				}
			}
		}

		// Cycle index
		if (m_twiddle.index > 2) m_twiddle.index = 0;

		// Execute current optimization step
		if (m_twiddle.isIncrease)
		{
			m_twiddle.k[m_twiddle.index] += m_twiddle.dk[m_twiddle.index];
		}
		else
		{
			m_twiddle.k[m_twiddle.index] -= m_twiddle.dk[m_twiddle.index];
		}

		// Output gain
		m_Kp = m_twiddle.k[0];
		m_Ki = m_twiddle.k[1];
		m_Kd = m_twiddle.k[2];

		// Debug information
		std::cout << "Twiddle updated: (k_p, k_i, k_d) = (" << m_twiddle.k[0] << ", " << m_twiddle.k[1] << ", " << m_twiddle.k[2] << ")" << std::endl;
	}
	else
	{
		// Debug information
		std::cout << "Twiddle terminated: (k_p, k_i, k_d) = (" << m_twiddle.k[0] << ", " << m_twiddle.k[1] << ", " << m_twiddle.k[2] << ")" << std::endl;
		std::cout << "        |dk| = |(" << m_twiddle.dk[0] << ", " << m_twiddle.dk[1] << ", " << m_twiddle.dk[2] << ")| = " << fabs(m_twiddle.dk[0]) + fabs(m_twiddle.dk[1]) + fabs(m_twiddle.dk[2]) << " < " << m_twiddle.threshold << std::endl;
	}

	// Debug information
	std::cout << " Twiddle best error: " << m_twiddle.bestError << std::endl;
}


