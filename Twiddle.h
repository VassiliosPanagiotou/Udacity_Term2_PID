#ifndef TWIDDLE_H
#define TWIDDLE_H

class STwiddle
{
  public:
	  STwiddle();
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
	  double threshold;
	  // Flag which indicates initial state
	  bool isInitial;
	  // Best error so far
	  double bestError;
};

#endif /* TWIDDLE_H */
