// not sure where i should implement this yet, but should take generated PWM
// signals (based on commanded wheel speeds) and do preliminary adjustment
// based on (calibrated) encoder data - i.e., if commanding wheel speed ratio
// doesn't match encoder data (e.g., trying to command all wheels to 50%  max,
// but one wheel moves faster/slower than others), scaling based purely on 
// encoder : pwm ratio

// may omit this if pi controller works well enough, but having chained
// controllers here might be nice
