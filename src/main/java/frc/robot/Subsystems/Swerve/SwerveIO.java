package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;
import swervelib.SwerveDrive;

public interface SwerveIO {
  @AutoLog
  public static class SwerveIOInputs {
    public Pose2d pose = new Pose2d();
    public ChassisSpeeds speeds = new ChassisSpeeds();
    public ChassisSpeeds targetSpeeds = new ChassisSpeeds();
    public SwerveModuleState[] swerveModuleStates;
    public SwerveModuleState[] swerveModuleDesiredStates;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveIOInputs inputs) {}

  public default void setMaxSpeed(double speed) {}

  public default double getMaxSpeed() {
    return 0;
  }

  public default void setMaxTurnSpeed(double speed) {}

  public default double getMaxTurnSpeed() {
    return 0;
  }

  public default void zeroGyro() {}

  public default double getGyro() {
    return 0;
  }

  public default Pose2d getPose() {
    return new Pose2d();
  }

  public default void resetPose(Pose2d pose) {}

  public default ChassisSpeeds getSpeeds() {
    return new ChassisSpeeds();
  }

  public default void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {}

  public default void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {}

  public default void driveRobotRelative(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {}

  public default SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    };
  }

  public default SwerveModuleState[] getModuleDesiredStates() {
    return new SwerveModuleState[] {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    };
  }

  public default SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
  }

  public default ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    return new ChassisSpeeds();
  }

  public default double getMaximumChassisVelocity() {
    return 0;
  }

  public default double getMaximumChassisAngularVelocity() {
    return 0;
  }

  public default void addVisionReading(
      Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {}

  public default SwerveDrive getSwerveDrive() {
    return null;
  }
}
