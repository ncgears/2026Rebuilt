package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.function.DoubleSupplier;

// ### slew rate limiter
// ### https://github.com/frc6995/Robot-2024/blob/main/src/main/java/frc/robot/util/drive/AsymmetricSlewRateLimiter.java

/** An extension of DoubleSupplier designed for pre-processing driver controller axis inputs. */
public class InputAxis implements DoubleSupplier {
  DoubleSupplier m_supplier;
  double deadband = 0;
  SlewRateLimiter limiter = new SlewRateLimiter(100,-100,0);
  boolean square;
  double multiplier = 1;

  double outputValue;
  String name;

  /**
   * Creates a new input axis wrapper for the provided supplier.
   *
   * @param name Human-readable name for logging/debugging.
   * @param supplier Input supplier to read.
   */
  public InputAxis(String name, DoubleSupplier supplier) {
    this.name = name;
    m_supplier = supplier;
  }

  // @Override
  // public String configureLogName() {
  //     return name;
  // }

  // private double inputValue() {
  //   return m_supplier.getAsDouble();
  // }

  /**
   * Applies an asymmetric slew rate limiter.
   *
   * @param forward Forward rate limit.
   * @param back Reverse rate limit.
   * @return This axis for chaining.
   */
  public InputAxis withSlewRate(double forward, double back) {
    limiter = new SlewRateLimiter(forward, back,0);
    return this;
  }

  /**
   * Applies a symmetric slew rate limiter.
   *
   * @param rate Rate limit in both directions.
   * @return This axis for chaining.
   */
  public InputAxis withSlewRate(double rate) {
    limiter = new SlewRateLimiter(rate,-rate,0);
    return this;
  }

  /**
   * Applies a scale multiplier to the axis output.
   *
   * @param multiplier Scalar multiplier.
   * @return This axis for chaining.
   */
  public InputAxis withMultiplier(double multiplier) {
    this.multiplier = multiplier;
    return this;
  }

  /**
   * Sets the deadband applied to the axis input.
   *
   * @param deadband Deadband amount.
   * @return This axis for chaining.
   */
  public InputAxis withDeadband(double deadband) {
    this.deadband = deadband;
    return this;
  }

  /**
   * Enables or disables squaring of the input while keeping sign.
   *
   * @param square True to square the input.
   * @return This axis for chaining.
   */
  public InputAxis withSquaring(boolean square) {
    this.square = square;
    return this;
  }

  /**
   * Inverts the current multiplier sign when requested.
   *
   * @param invert True to invert axis output.
   * @return This axis for chaining.
   */
  public InputAxis withInvert(boolean invert) {
    this.multiplier = Math.abs(this.multiplier) * (invert ? -1 : 1);
    return this;
  }

  /** Resets the slew rate limiter to the current filtered input value. */
  public void resetSlewRate() {
    double value = m_supplier.getAsDouble();
    value *= multiplier;
    value = MathUtil.applyDeadband(value, deadband);
    if (this.square) {
      value = Math.copySign(value * value, value);
    }
    limiter.reset(value);
  }

  /**
   * Returns the processed axis value.
   *
   * @return Filtered output value.
   */
  @Override
  public double getAsDouble() {
    double value = m_supplier.getAsDouble();
    value *= multiplier;
    value = MathUtil.applyDeadband(value, deadband);
    if (this.square) {
      value = Math.copySign(value * value, value);
    }
    value = limiter.calculate(value);
    outputValue = value;
    return value;
  }
}
