package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
    @AutoLog
    public static class SwerveIOInputs {
        // TODO - Implement
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(SwerveIOInputs inputs) {}

    
}
