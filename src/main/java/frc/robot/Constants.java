// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static boolean isBlueAlliance = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class Swerve {
    public static final PIDConstants translationConstants = new PIDConstants(5, 0.0, 0.0);
    public static final PIDConstants rotationConstants = new PIDConstants(5, 0.0, 0.0);
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.07;
    public static final double LEFT_Y_DEADBAND = 0.07;
    public static final double RIGHT_X_DEADBAND = 0.07;
    public static final double TURN_CONSTANT = 6;
    public static final int DRIVER_LEFT = 0;
    public static final int DRIVER_RIGHT = 1;
    public static final int DRIVER_BUTTONS = 2;
    public static final int CO_DRIVER_LEFT = 3;
    public static final int CO_DRIVER_RIGHT = 4;
    public static final int CO_DRIVER_BUTTONS = 5;
  }

  public static class VisionConstants {
    public static final boolean USE_VISION = true;

    // Transform Camera Coordinates to Robot Coordinates. Based on camera mounting
    // position.
    public static final Matrix<N4, N4> transformFrontLeftToRobot = new Transform3d(0.1524, 0.3556, 0,
        new Rotation3d(0, 0, Math.toRadians(-35.0))).toMatrix();

    public static final Transform3d camOneTransform = new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.toRadians(0)));

    public static final Transform3d triceratopsTransform = new Transform3d(0, 0.5379, 0,
        new Rotation3d(0, 0, Math.toRadians(0)));

    // Position of the AprilTag in Tag Coordinates.
    public static final Matrix<N4, N1> referenceTagPosition = new Matrix<>(Nat.N4(), Nat.N1(),
        new double[] { 0, 0, 0, 1 });

    // Constant Distance from Tag to Pole (6in in meters)
    public static final double distanceToPole = 0.1524;

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.2);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.2);
  }
}
