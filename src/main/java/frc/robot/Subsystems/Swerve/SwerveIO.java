package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  public static class SwerveIOInputs {
    // TODO - Implement
    public enum DriveState {None, Manual, AlignReefLeft, AlignReefRight, AlignProcessor, AlignSource}; 
    public DriveState driveState = DriveState.None;

    public Pose2d pose = new Pose2d();
    public ChassisSpeeds speeds = new ChassisSpeeds();
    public ChassisSpeeds targetSpeeds = new ChassisSpeeds();
    public SwerveModuleState[] swerveModuleStates;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveIOInputs inputs) {}
}
