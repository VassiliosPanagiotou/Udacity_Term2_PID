#ifndef PID_H
#define PID_H

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

  //Step Counter
  unsigned m_StepCount;


  struct STwiddle
  {
	  STwiddle(double Kp, double Ki, double Kd, double thres = 0.01) {
	  
		  k[0] = Kp;
		  k[1] = Ki;
		  k[2] = Kd;
		  dk[0] = fabs(Kp) / 10.0;
		  dk[1] = fabs(Ki) / 10.0;
		  dk[2] = fabs(Kd) / 10.0;

		  index = 0;
		  isIncrease = true;
		  threshold = thres;
		  isInitial = true;
		  bestError = 100000000.0;
	  };

	  virtual ~STwiddle();

	  // Optimization gain
	  double k[3];
	  // Optimization gain increments
	  double dk[3];
	  // Gain index currently optimized 
	  unsigned index;
	  // Flag increase of gain (true) or decrease of gain (false)
	  bool isIncrease;
	  // Optimization threshold
	  const double threshold;
	  // Flag which indicates initial state
	  bool isInitial;
	  // Best error so far
	  double bestError;
  } m_twiddle;

   /*
  * Initialize Errors - used after Twiddle run
  */
  void InitError(double Kp, double Ki, double Kd);

  /*
  * Constructor
  */
  PID();
};

#endif /* PID_H */
