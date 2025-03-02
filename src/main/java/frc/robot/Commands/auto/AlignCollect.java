package frc.robot.Commands.auto;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.States.AlignState;
import frc.robot.Constants;
import frc.robot.Commands.complex.CollectCoral;
import frc.robot.Commands.complex.Intake;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.RangeAlignSource;
import frc.robot.Subsystems.Vision.AlignVision;

public class AlignCollect extends Command {
    public AlignCollect() {}

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new Intake());
        Swerve.getInstance().setDriveState(DriveState.AlignSource);
    }
    @Override
    public boolean isFinished() {
        if (Constants.currentMode == Constants.simMode) {
            return true;
        }
        return CoralRollers.getInstance().HasCoral();
    }
    public void end(boolean interrupted)
    {
        Swerve.getInstance().setDriveState(DriveState.Manual);
    }

}
