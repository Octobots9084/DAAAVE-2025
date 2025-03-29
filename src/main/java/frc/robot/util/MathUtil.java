package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MathUtil {
    /**
     * Wraps a circular angle value to within one circle.
     * <p>
     * Customizable number of angle units per circle. Angle is equivalent and
     * wrapped to the
     * positive [0, fullCircle] range.
     *
     * @param angle
     *            the raw angle units to wrap.
     * @param fullCircle
     *            the number of angle units to be one circle.
     * @return the wrapped angle in corresponding angle units.
     */
    public static double wrapToCircle(double angle, double fullCircle) {
        angle %= fullCircle;
        return angle < 0 ? fullCircle + angle : angle;
    }

    /**
     * Checks if a value is within a certain tolerance of a target. Directions
     * irrelevant.
     *
     * @param value
     *            the current value for which to check.
     * @param target
     *            the target to check the value against.
     * @param tolerance
     *            the tolerance (positive and negative directions) around the
     *            target that is
     *            acceptable error for the value to be "within tolerance".
     * @return if the value is within tolerance of the target.
     */
    public static boolean isWithinTolerance(double value, double target, double tolerance) {
        return Math.abs(value - target) < tolerance;
    }

    /**
     * Wraps an angle to the unit circle, then clamps to [-pi, pi]
     * such that the angle loops across the range if it exceeds it.
     * 
     * @param angle
     *            The unbounded input angle.
     * @return The looped, clamped angle.
     */
    public static double clampAngle(double angle) {
        angle = MathUtil.wrapToCircle(angle, 2 * Math.PI);
        if (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Fits value between -1 and 1 but eliminates noise between the deadband.
     * <p>
     * Generally used for eliminating noise in joystick readings by setting the
     * output to zero when
     * within a certain deadband amount of zero. Non-joystick values are also
     * limited between -1 and 1
     * so as to use in a motor set.
     *
     * @param val
     *            the value to fit inside the valid range and outside the
     *            deadband.
     * @param deadband
     *            the amount of tolerance around zero in which values are set
     *            to zero.
     * @return the value fitted to the range.
     * @since 0.1.0
     */
    public static double fitDeadband(double val, double deadband) {
        if (Math.abs(val) >= deadband) {
            if (val > 0) {
                if (val >= 1) {
                    return 1;
                } else {
                    return (val - deadband) / (1 - deadband);
                }
            } else if (val < 0) {
                if (val <= -1) {
                    return -1;
                } else {
                    return (val + deadband) / (1 - deadband);
                }
            }
        }
        return 0;
    }

    /**
     * Flips an angle over the y axis.
     *
     * @param angle
     *            The angle in degrees.
     * @return The flipped angle.
     */
    public static double flipAngleOverYAxis(double angle) {
        double radians = Math.toRadians(angle);
        double sin = Math.sin(radians);
        double cos = Math.cos(radians);
        double mirroredCos = -cos;
        double mirroredRadians = Math.atan2(sin, mirroredCos);
        double mirroredAngle = Math.toDegrees(mirroredRadians);
        return mirroredAngle;
    }

    /**
     * Exponentiates a number while preserving the sign of the number
     * pre-exponentiation.
     *
     * @param a
     *            The number to be exponentiated.
     * @param b
     *            The power with which to exponentiate.
     * @return The exponentiated number w/ preserved sign.
     */
    public static double powPreserveSign(double a, double b) {
        final double sign = Math.signum(a);
        return sign * Math.pow(Math.abs(a), b);
    }

    /**
     * Takes a vector within the unit square and converts it to the appropriate
     * vector on a unit circle scaled by the given scale factor.
     *
     * @param x
     *            The vector's x component (between -1 and 1).
     * @param y
     *            The vector's y component (between -1 and 1).
     * @param vectorScaleFactor
     *            The factor with which to scale the base vector by.
     * @return An array containing the x and y vector components of the scaled
     *         vector (as index 0 and 1 respectively).
     */
    public static double[] circleVectorFromSquare(double x, double y, double vectorScaleFactor) {
        double angle = Math.atan2(y, x);
        double maxDist = (0.375 + ((Math.asin(Math.sin(2 * (angle + 0.785398163397) + 1.57079632679)))
                / (Math.sin(2 * (angle + 0.785398163397) + 1.57079632679) * 1.57079632679)));
        double scaleRatio = vectorScaleFactor * Math.sqrt(x * x + y * y) / maxDist;
        return new double[] { x * scaleRatio, y * scaleRatio };
    }

    public static ChassisSpeeds limitXAndYAcceleration(ChassisSpeeds targetChassisSpeeds,
            ChassisSpeeds currentChassisSpeeds, double maxAccelerationX, double maxAccelerationY, double loopTime) {
        double targetAccelerationX = (targetChassisSpeeds.vxMetersPerSecond - currentChassisSpeeds.vxMetersPerSecond)
                / loopTime;
        double targetAccelerationY = (targetChassisSpeeds.vyMetersPerSecond - currentChassisSpeeds.vyMetersPerSecond)
                / loopTime;

        if (Math.abs(targetAccelerationX) > maxAccelerationX) {
            targetAccelerationX = maxAccelerationX * Math.signum(targetAccelerationX);
        }

        if (Math.abs(targetAccelerationY) > maxAccelerationY) {
            targetAccelerationY = maxAccelerationY * Math.signum(targetAccelerationY);
        }

        return new ChassisSpeeds(currentChassisSpeeds.vxMetersPerSecond + targetAccelerationX * loopTime,
                currentChassisSpeeds.vyMetersPerSecond + targetAccelerationY * loopTime,
                targetChassisSpeeds.omegaRadiansPerSecond);

    }

    public static ChassisSpeeds limitVectorAcceleration(
            ChassisSpeeds targetChassisSpeeds,
            ChassisSpeeds currentChassisSpeeds,
            double maxAcceleration, // Single max acceleration limit
            double loopTime) {

        // Calculate acceleration components
        double accelX = (targetChassisSpeeds.vxMetersPerSecond - currentChassisSpeeds.vxMetersPerSecond) / loopTime;
        double accelY = (targetChassisSpeeds.vyMetersPerSecond - currentChassisSpeeds.vyMetersPerSecond) / loopTime;

        // Compute acceleration vector magnitude
        double accelMagnitude = Math.sqrt(accelX * accelX + accelY * accelY);

        // If acceleration exceeds max allowed, scale it down while preserving direction
        if (accelMagnitude > maxAcceleration) {
            double scale = maxAcceleration / accelMagnitude;
            accelX *= scale;
            accelY *= scale;
        }

        // Compute new velocities
        double newVx = currentChassisSpeeds.vxMetersPerSecond + accelX * loopTime;
        double newVy = currentChassisSpeeds.vyMetersPerSecond + accelY * loopTime;

        return new ChassisSpeeds(newVx, newVy, targetChassisSpeeds.omegaRadiansPerSecond);
    }
}
