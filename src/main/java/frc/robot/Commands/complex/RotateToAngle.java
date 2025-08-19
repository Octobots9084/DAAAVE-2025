package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;

public class RotateToAngle extends Command {
    public RotateToAngle (int angle) {
        AlignVision.getInstance().getGyroRotationPIDController().calculate(Swerve.getInstance().getPose().getRotation().getRadians(), angle);
    }

    @Override
    public boolean isFinished () {
        return Swerve.getInstance().getPose().getRotation().getDegrees() == 0.0;
    }
}