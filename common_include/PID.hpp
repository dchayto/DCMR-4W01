/*									PID.hpp									//
	structure for generic PID controller; to be instantiated by velocity
	PIDs in mec wheel controller and motor controller
*/

class PID	{
public:
	PID(double kp, double ki, double kd, double i_max = 9999)
	 : kp_{kp}
	 , ki_{ki}
	 , kd_{kd}
	 , max_itgl_{ (i_max >= 0.0 ? i_max : -i_max) }
	{} // </constructor>

	double correct(double err, double dt)	{	
		// note: expecting dt in s
		e_itgl_ += err*dt;		// add error signal to summation
		de_ = err - prev_err_;	// calculate differential error
		prev_err_ = err;	

		if (e_itgl_ > max_itgl_) 		e_itgl_ = max_itgl_;
		if (e_itgl_ < -max_itgl_) 		e_itgl_ = -max_itgl_;

		// adjust control signal
		return (err*kp_) + (ki_*e_itgl_) + (kd_ * de_ / dt);
	}

	void reset()	{
		e_itgl_ 	= 0.0;
		de_ 		= 0.0;
		prev_err_	= 0.0;
	}

private:	
	const double kp_;
	const double ki_;
	const double kd_;
	const double max_itgl_;		// saturation limit (optional)

	double e_itgl_ { 0.0 }; 	// storing "integral" of error signal
	double de_ { 0.0 };			// err - prev err; for d gain compute
	double prev_err_ { 0.0 };
}; // </PID>
