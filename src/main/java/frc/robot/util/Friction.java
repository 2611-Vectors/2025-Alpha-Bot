package frc.robot.util;

import static frc.robot.Constants.FrictionConstants.*;

public class Friction {

  // Linear interpolation function
  public static double lerp(double start, double end, double t) {
    return (1 - t) * start + t * end;
  }

  // Normalize the degree to be within 0 and 360
  public static double normalizeDegree(double degree) {
    degree = degree % 360;
    if (degree < 0) {
      degree += 360; // Handle negative degrees
    }
    return degree;
  }

  // Main interpolation function that handles the degree-based interpolation
  public static double getFriction(double degree) {
    degree = normalizeDegree(degree); // Normalize the degree

    // Define the points
    double f = MU_FORWARD; // Example value for f
    double r1 = MU_RIGHT; // Example value for r1
    double b = MU_BACKWARD; // Example value for b
    double l = MU_LEFT; // Example value for l

    // Normalize the degree to a value between 0 and 1
    double t;

    if (degree >= 0 && degree < 90) {
      t = degree / 90;
      return lerp(f, l, t);
    } else if (degree >= 90 && degree < 180) {
      t = (degree - 90) / 90;
      return lerp(l, b, t);
    } else if (degree >= 180 && degree < 270) {
      t = (degree - 180) / 90;
      return lerp(b, r1, t);
    } else if (degree >= 270 && degree < 360) {
      t = (degree - 270) / 90;
      return lerp(r1, f, t);
    } else {
      // Should not reach here due to normalization, but can throw an exception for safety
      throw new IllegalArgumentException("Degree normalization failed.");
    }
  }
}
