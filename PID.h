#ifndef PID_H
#define PID_H

class PID {
public:
	// Ctor
	PID(double Kp = 0.3, double Ki = 0.0001, double Kd = 1.0, bool runTwiddle = false);
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

  /*
  * Constructor
  */
  PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
