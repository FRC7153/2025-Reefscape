package frc.robot.util.math;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;

/**
 * Calculates derivative of a value with respect to time.
 */
public class DerivativeCalculator {
  // Filter to "smooth" the derivative
  private final LinearFilter filter;

  private double lastValue;
  private double lastTS;
  private boolean firstRun = true;

  /**
   * @param filterTaps Number of taps for the linear filter that smooths the derivative.
   */
  public DerivativeCalculator(int filterTaps) {
    filter = LinearFilter.movingAverage(filterTaps);
  }

  /**
   * @param newValue The next value.
   * @return The derivative.
   */
  public double calculate(double newValue) {
    double ts = Timer.getFPGATimestamp();

    if (!firstRun) {
      // Only calculate if not the first run
      double delta = (newValue - lastValue) / (ts - lastTS);
      filter.calculate(delta);
    } else {
      firstRun = false;
    }

    lastValue = newValue;
    lastTS = ts;

    return filter.lastValue(); // 0.0 if first run
  }

  /**
   * @return The last derivative calculated.
   */
  public double lastValue() {
    return filter.lastValue();
  }
}
