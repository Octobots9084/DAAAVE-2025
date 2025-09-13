package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.util.MathUtil;

public class RotateToAngle extends Command {
    private int angle;
    public RotateToAngle (int angle) {
        //PASS IN DEGREES FOOL
        this.angle = angle;
    }

    @Override
    public void initialize () {
        AlignVision.getInstance().getGyroRotationPIDController().calculate(Swerve.getInstance().getPose().getRotation().getRadians(), angle);
    }

    @Override
    public boolean isFinished () {
        return MathUtil.isWithinTolerance(Swerve.getInstance().getPose().getRotation().getDegrees(), angle, 2);//its in degrees
    }
}