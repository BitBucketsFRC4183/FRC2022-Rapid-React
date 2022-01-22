package frc.robot.utils;

import edu.wpi.first.math.*;

public class MathUtils {

  // TODO: Finish documenting the unit converter.
  /**
   * unitConverter: a function to dynamically convert between units.
   *
   * Make sure inputUnit and outputUnit represent the same value. E.g. if you want
   * to convert degrees to radians both inputUnit and outputUnit should represent
   * 1 revolution.
   *
   * Under no circumstances should inputUnit or outputUnit be 0.
   *
   * The unit converter works by multiplying the inputQuantity by the ratio
   * between outputUnit and inputUnit.
   *
   * @param inputQuantity How much of a given unit you want to input.
   *
   * @param inputUnit     The specific unit you want to input.
   *
   * @param outputUnit    The unit which the converter will output.
   */
  public static double unitConverter(double inputQuantity, double inputUnit, double outputUnit) {
    return inputQuantity * (outputUnit / inputUnit);
  }

  /**
   * Clamps the input between two values, then returns the new value.
   *
   * @param input The number to clamp.
   * @param min   The minium allowed output.
   * @param max   The maximum allowed output.
   */
  public static double clamp(double input, double min, double max) {
    return input < min ? min : input > max ? max : input;
  }

  /**
   * Takes the average of 2 numbers.
   */
  public static double average(double value, double otherValue) {
    return (value + otherValue) / 2;
  }

  public static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
