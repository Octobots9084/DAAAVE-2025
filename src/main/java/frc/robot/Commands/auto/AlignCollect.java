package frc.robot.Commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.States.AlignState;
import frc.robot.Commands.complex.CollectCoral;
import frc.robot.Commands.complex.Intake;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.RangeAlignSource;
import frc.robot.Subsystems.Vision.AlignVision;

public class AlignCollect extends Command {
    public AlignCollect() {}

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new Intake());

    }

    @Override
    public void execute() {
        Swerve.getInstance().driveRobotRelative(RangeAlignSource.getInstance().getAlignChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return CoralRollers.getInstance().HasCoral();
    }

}
