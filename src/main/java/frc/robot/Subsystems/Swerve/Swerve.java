// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Basic simulation of a swerve subsystem with the methods needed by PathPlanner */
public class Swerve extends SubsystemBase {
  SwerveIOSystem io;
  private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
  private static Swerve INSTANCE = null;

  public static enum DriveState {
    None,
    Manual,
    AlignReefLeft,
    AlignReefRight,
    AlignProcessor,
    AlignSource
  };

  private DriveState driveState = DriveState.None;

  public boolean isAlignedToSource;
  public boolean isAlignedToCoralRight;
  public boolean isAlignedToCoralLeft;
  public boolean isAlignedToProcessor;

  public static Swerve getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Swerve();
    }

    return INSTANCE;
  }

  public Swerve() {
    this.io = new SwerveIOSystem();
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getSpeeds,
          (speeds, feedforwards) -> driveRobotRelative(speeds),
          new PPHolonomicDriveController(
              Constants.Swerve.translationConstants, Constants.Swerve.rotationConstants),
          config,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);

    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
  }

  public SwerveIOSystem getIo() {
    return io;
  }

  public void zeroGyro() {
    this.io.zeroGyro();
  }

  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    Logger.processInputs("Swerve Drive", inputs);
  }

  public double getGyro() {
    return this.io.getGyro();
  }

  public Pose2d getPose() {
    return this.io.getPose();
  }

  public void resetPose(Pose2d pose) {
    // System.out.println(pose);
    this.io.resetPose(pose);
  }

  public void setDriveState(DriveState state) {
    driveState = state;
    Logger.recordOutput("DriveState", state);
  }

  public DriveState getDriveState() {
    return driveState;
  }

  public ChassisSpeeds getSpeeds() {
    return this.io.getSpeeds();
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    this.io.driveFieldRelative(fieldRelativeSpeeds);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    SmartDashboard.putString("Chassis speeds", robotRelativeSpeeds.toString());
    this.io.driveRobotRelative(robotRelativeSpeeds);
  }

  public SwerveModuleState[] getModuleStates() {
    return this.io.getModuleStates();
  }

  public SwerveModuleState[] getModuleDesiredStates() {
    return this.io.getModuleDesiredStates();
  }

  public SwerveModulePosition[] getPositions() {
    return this.io.getPositions();
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX Heading X to calculate angle of the joystick.
   * @param headingY Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    return run(
        () -> {
          double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
          double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
          // Make the robot move
          driveFieldRelative(
              this.io.getTargetSpeeds(
                  xInput, yInput, headingX.getAsDouble(), headingY.getAsDouble()));
        });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(
        () -> {
          // Make the robot move
          this.io.driveRobotRelative(
              new Translation2d(
                  translationX.getAsDouble() * this.io.getMaximumChassisVelocity(),
                  translationY.getAsDouble() * this.io.getMaximumChassisVelocity()),
              angularRotationX.getAsDouble() * this.io.getMaximumChassisAngularVelocity(),
              true,
              false);
        });
  }

  public void addVisionReading(
      Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    this.io.addVisionReading(robotPose, timestamp, visionMeasurementStdDevs);
  }
}
