package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import swervelib.SwerveDrive;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveIOSystem implements SwerveIO {
  private SwerveDrive swerveDrive;
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  double maximumSpeed = 12;
  double maxTurnSpeed = 5;
  private Field2d field = new Field2d();
  SlewRateLimiter xFilterL4FieldRelative = new SlewRateLimiter(3);
  SlewRateLimiter yFilterL4FieldRelative = new SlewRateLimiter(3);

  SlewRateLimiter xFilterL3FieldRelative = new SlewRateLimiter(6);
  SlewRateLimiter yFilterL3FieldRelative = new SlewRateLimiter(6);

  SlewRateLimiter xFilterL4RobotRelative = new SlewRateLimiter(3);
  SlewRateLimiter yFilterL4RobotRelative = new SlewRateLimiter(3);

  SlewRateLimiter xFilterL3RobotRelative = new SlewRateLimiter(6);
  SlewRateLimiter yFilterL3RobotRelative = new SlewRateLimiter(6);

  public SwerveIOSystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory)
          .createSwerveDrive(maximumSpeed, new Pose2d(3.1, 4, new Rotation2d(0)));
      swerveDrive.setHeadingCorrection(
          false); // Heading correction should only be used while controlling the robot via
      // angle.
      swerveDrive.setCosineCompensator(
          !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation
      // for simulations since it causes discrepancies not seen in real life.
      swerveDrive.setAngularVelocityCompensation(true, true, 0.07);
    } catch (IOException e) {
      e.printStackTrace();
    }

    SmartDashboard.putData("Field", field);
  }

  public void setMaxSpeed(double speed) {
    maximumSpeed = speed;
  }

  public double getMaxSpeed() {
    return maximumSpeed;
  }

  public void setMaxTurnSpeed(double speed) {
    maxTurnSpeed = speed;
  }

  public double getMaxTurnSpeed() {
    return maxTurnSpeed;
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public double getGyro() {
    return swerveDrive.getGyroRotation3d().getZ();
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public ChassisSpeeds getSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    if (Elevator.getInstance().isAtState(ElevatorStates.LEVEL4, 5)) {
      robotRelativeSpeeds = new ChassisSpeeds(
          xFilterL4RobotRelative.calculate(robotRelativeSpeeds.vxMetersPerSecond),
          yFilterL4RobotRelative.calculate(robotRelativeSpeeds.vyMetersPerSecond),
          robotRelativeSpeeds.omegaRadiansPerSecond);
    } else if (Elevator.getInstance().isAtState(ElevatorStates.LEVEL3, 5)) {
      robotRelativeSpeeds = new ChassisSpeeds(
          xFilterL3RobotRelative.calculate(robotRelativeSpeeds.vxMetersPerSecond),
          yFilterL3RobotRelative.calculate(robotRelativeSpeeds.vyMetersPerSecond),
          robotRelativeSpeeds.omegaRadiansPerSecond);
    }
    swerveDrive.drive(robotRelativeSpeeds);
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    if (Elevator.getInstance().isAtState(ElevatorStates.LEVEL4, 5) && Constants.currentMode == Mode.REAL) {
      fieldRelativeSpeeds = new ChassisSpeeds(
          xFilterL4FieldRelative.calculate(fieldRelativeSpeeds.vxMetersPerSecond),
          yFilterL4FieldRelative.calculate(fieldRelativeSpeeds.vyMetersPerSecond),
          fieldRelativeSpeeds.omegaRadiansPerSecond);
    } else if (Elevator.getInstance().isAtState(ElevatorStates.LEVEL4, 5) && Constants.currentMode == Mode.REAL) {
      fieldRelativeSpeeds = new ChassisSpeeds(
          xFilterL3FieldRelative.calculate(fieldRelativeSpeeds.vxMetersPerSecond),
          yFilterL3FieldRelative.calculate(fieldRelativeSpeeds.vyMetersPerSecond),
          fieldRelativeSpeeds.omegaRadiansPerSecond);
    } else {
        fieldRelativeSpeeds = new ChassisSpeeds(
          fieldRelativeSpeeds.vxMetersPerSecond,
          fieldRelativeSpeeds.vyMetersPerSecond,
          fieldRelativeSpeeds.omegaRadiansPerSecond);
    }

    swerveDrive.driveFieldOriented(fieldRelativeSpeeds);
  }

  public SwerveModuleState[] getModuleStates() {
    return swerveDrive.getStates();
  }

  public SwerveModulePosition[] getPositions() {
    return swerveDrive.getModulePositions();
  }

  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    return swerveDrive.swerveController.getTargetSpeeds(
        xInput,
        yInput,
        headingX,
        headingY,
        swerveDrive.getYaw().getRadians(),
        swerveDrive.getMaximumChassisVelocity());
  }

  public double getMaximumChassisVelocity() {
    return swerveDrive.getMaximumChassisVelocity();
  }

  public double getMaximumChassisAngularVelocity() {
    return swerveDrive.getMaximumChassisAngularVelocity();
  }

  public void addVisionReading(
      Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    swerveDrive.addVisionMeasurement(new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d(this.getGyro())),
        timestamp, visionMeasurementStdDevs);
  }

  @Override
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  // public SwerveModuleState[] getModuleDesiredStates() {
  // return swerveDrive.getDesiredStates();
  // }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    // TODO - Implement
    inputs.pose = this.getPose();
    inputs.speeds = this.getSpeeds();

    inputs.swerveModuleStates = this.getModuleStates();
    inputs.swerveModuleDesiredStates = this.getModuleDesiredStates();
    inputs.gyroAngleRadians = swerveDrive.getGyro().getRotation3d().toRotation2d().getRadians();
  }
}
