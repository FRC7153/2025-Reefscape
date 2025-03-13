package frc.robot.util.math;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.BuildConstants;

public class Triangle2d {
  private final Translation2d a, b, c;
  private final double area;

  public Triangle2d(Translation2d a, Translation2d b, Translation2d c) {
    this.a = a;
    this.b = b;
    this.c = c;

    area = calculateTriangularArea(a, b, c);
  }

  /**
   * Checks if a point is in a triangle.
   * @param pt
   * @return
   */
  public boolean containsPoint(Translation2d pt) {
    double subTriangleArea = 
      calculateTriangularArea(a, b, pt) +
      calculateTriangularArea(a, c, pt) +
      calculateTriangularArea(b, c, pt);

    return Math.abs(subTriangleArea - area) <= BuildConstants.EPSILON;
  }

  /**
   * Calculate triangular area from 3 2d points.
   * @param a
   * @param b
   * @param c
   * @return The area.
   */
  private static double calculateTriangularArea(Translation2d a, Translation2d b, Translation2d c) {
    return 0.5 * Math.abs(
      a.getX() * (b.getY() - c.getY()) + 
      b.getX() * (c.getY() - a.getY()) + 
      c.getX() * (a.getY() - b.getY()));
  }
}
