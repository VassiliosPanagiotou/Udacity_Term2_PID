#ifndef PID_H
#define PID_H

#include "Twiddle.h"

class PID {
public:
	// Ctor
	PID(double Kp = 0.2, double Ki = 0.0004, double Kd = 3.0, bool runTwiddle = false);
	// Dtor
	virtual ~PID();

	///////////////
	// Given the cross track error, update PID error variables and return steering control value
	///////////////
	double getSteerValue(double cte);
	
private:
  /*
  * Errors
  */
  double m_p_error;
  double m_i_error;
  double m_d_error;

  /*
  * Coefficients
  */ 
  double m_Kp;
  double m_Ki;
  double m_Kd;

  /*
  * Flag for running Twiddle
  */
  bool m_RunTwiddle;

  // Step Counter
  unsigned m_StepCount;
  
  // Twiddle helper
  STwiddle m_twiddle;

   /*
  * Initialize Errors - used after Twiddle run
  */
  void InitError();

  // Runs twiddle
  void RunTwiddle(double error);
  /*
  * Constructor
  */
  PID();
};

#endif /* PID_H */
