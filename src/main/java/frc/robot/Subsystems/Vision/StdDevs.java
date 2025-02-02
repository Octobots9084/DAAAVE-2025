package frc.robot.Subsystems.Vision;

public class StdDevs {
  private int count = 0; // Number of elements

  private double meanX = 0.0; // Mean of X values
  private double meanY = 0.0; // Mean of Y values
  private double meanRotation = 0.0; // Mean of rotation values

  private double m2X = 0.0; // Sum of squares of differences for X
  private double m2Y = 0.0; // Sum of squares of differences for Y
  private double m2Rotation = 0.0; // Sum of squares of differences for rotation

  /**
   * Updates the statistics with new values for X, Y, and rotation.
   *
   * @param x The new X value.
   * @param y The new Y value.
   * @param rotation The new rotation value.
   * @return An array containing the updated standard deviations for X, Y, and rotation.
   */
  public double[] update(double x, double y, double rotation) {
    count++;

    // Update X statistics
    double oldMeanX = meanX;
    meanX += (x - meanX) / count;
    m2X += (x - oldMeanX) * (x - meanX);

    // Update Y statistics
    double oldMeanY = meanY;
    meanY += (y - meanY) / count;
    m2Y += (y - oldMeanY) * (y - meanY);

    // Update rotation statistics
    double oldMeanRotation = meanRotation;
    meanRotation += (rotation - meanRotation) / count;
    m2Rotation += (rotation - oldMeanRotation) * (rotation - meanRotation);

    // Return the standard deviations
    return new double[] {
      getStandardDeviationX(), getStandardDeviationY(), getStandardDeviationRotation()
    };
  }

  /**
   * Returns the current standard deviation for X.
   *
   * @return The standard deviation for X, or 0.0 if no values have been added.
   */
  public double getStandardDeviationX() {
    if (count < 2) {
      return 0.0;
    }
    return Math.sqrt(m2X / count);
  }

  /**
   * Returns the current standard deviation for Y.
   *
   * @return The standard deviation for Y, or 0.0 if no values have been added.
   */
  public double getStandardDeviationY() {
    if (count < 2) {
      return 0.0;
    }
    return Math.sqrt(m2Y / count);
  }

  /**
   * Returns the current standard deviation for rotation.
   *
   * @return The standard deviation for rotation, or 0.0 if no values have been added.
   */
  public double getStandardDeviationRotation() {
    if (count < 2) {
      return 0.0;
    }
    return Math.sqrt(m2Rotation / count);
  }

  /**
   * Returns the current means for X, Y, and rotation.
   *
   * @return An array containing the means for X, Y, and rotation.
   */
  public double[] getMeans() {
    return new double[] {meanX, meanY, meanRotation};
  }

  /**
   * Returns the count of values added.
   *
   * @return The number of values.
   */
  public int getCount() {
    return count;
  }
}
