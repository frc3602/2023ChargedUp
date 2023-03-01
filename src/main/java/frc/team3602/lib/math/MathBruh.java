package frc.team3602.lib.math;

public class MathBruh {
  public static boolean between(double value, double minValue, double maxValue) {
    if (value >= minValue && value <= maxValue) {
      return true;
    } else {
      return false;
    }
  }
}
