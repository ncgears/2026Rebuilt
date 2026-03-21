package frc.robot.utils;

/**
 * Container for PID and Motion Magic gains used when configuring motor controller slots.
 */
public class PIDGains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int kIzone;
	public final double kPeakOutput;
    public final double kNeutralDeadband;
    public final double kCruise;
    public final double kAccel;
    public final int kSCurve;
	
    /**
     * Creates a new gains container for closed-loop and motion profile tuning.
     *
     * @param _kP Proportional gain.
     * @param _kI Integral gain.
     * @param _kD Derivative gain.
     * @param _kF Feed-forward gain.
     * @param _kIzone Integral zone.
     * @param _kPeakOutput Maximum controller output, typically 1.0.
     * @param _kNeutralDeadband Neutral deadband, typically 0.04.
     * @param _kCruise Motion Magic cruise value.
     * @param _kAccel Motion Magic acceleration value.
     * @param _kSCurve Motion Magic S-curve strength from 0 (off) to 8 (strong).
     */
	public PIDGains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput, double _kNeutralDeadband, double _kCruise, double _kAccel, int _kSCurve){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
        kNeutralDeadband = _kNeutralDeadband;
        kCruise = _kCruise;
        kAccel = _kAccel;
        kSCurve = _kSCurve;
	}
}
