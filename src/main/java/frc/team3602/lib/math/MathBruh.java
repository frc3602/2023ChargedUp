package frc.team3602.lib.math;

public class MathBruh {
  public static boolean between(double value, double minValue, double maxValue) {
    return value >= minValue && value <= maxValue || value <= minValue && value >= maxValue;
  }
}
