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
import frc.robot.Subsystems.Elevator.Elevator;

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
  SlewRateLimiter xFilterL4 = new SlewRateLimiter(3);
  SlewRateLimiter yFilterL4 = new SlewRateLimiter(3);

  SlewRateLimiter xFilterL3 = new SlewRateLimiter(6);
  SlewRateLimiter yFilterL3 = new SlewRateLimiter(6);

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
    List<Matter> matter = new ArrayList<Matter>();
    matter.add(new Matter(new Translation3d(0.1, 0, Elevator.getInstance().getPosition() * 0.01262), 20));

    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds,
        swerveDrive.getYaw());
    Translation2d newTranslation2d = SwerveMath.limitVelocity(
        new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond),
        swerveDrive.getFieldVelocity(), swerveDrive.getPose(), 0.02, 50, matter, swerveDrive.swerveDriveConfiguration);

    ChassisSpeeds newVelocityFieldRelative = new ChassisSpeeds(newTranslation2d.getX(), newTranslation2d.getY(),
        fieldRelativeSpeeds.omegaRadiansPerSecond);
    ChassisSpeeds newVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(newVelocityFieldRelative, swerveDrive.getYaw());

    swerveDrive.drive(newVelocity);
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    // - (Elevator.getInstance().getPosition() / 150.0) * 0.5);
    ChassisSpeeds limitedFieldRelativeSpeeds = new ChassisSpeeds(
        xFilterL4.calculate(fieldRelativeSpeeds.vxMetersPerSecond),
        yFilterL4.calculate(fieldRelativeSpeeds.vyMetersPerSecond),
        fieldRelativeSpeeds.omegaRadiansPerSecond);

    swerveDrive.driveFieldOriented(limitedFieldRelativeSpeeds);
  }

  public void driveRobotRelative(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    List<Matter> matter = new ArrayList<Matter>();
    matter.add(new Matter(new Translation3d(0.1, 0, Elevator.getInstance().getPosition() * 0.01262), 20));

    Translation2d newTranslation2d = SwerveMath.limitVelocity(
        translation,
        swerveDrive.getFieldVelocity(), swerveDrive.getPose(), 0.02, 50, matter, swerveDrive.swerveDriveConfiguration);

    swerveDrive.drive(newTranslation2d, rotation, fieldRelative, isOpenLoop);
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
    swerveDrive.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
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
