package frc.robot.Commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;

public class testAlgae extends Command {
    ReefTargetOrientation targetOrientation;

    public testAlgae(ReefTargetOrientation targetOrientation) {
        this.targetOrientation = targetOrientation;
    }

    @Override
    public void initialize() {
        AlignVision.setReefOrientation(targetOrientation);
    }

    @Override
    public void execute() {
        Swerve.getInstance().driveRobotRelative(AlignVision.getInstance().getAlignChassisSpeeds(AlignState.Reef));
    }

    @Override
    public boolean isFinished() {
        return AlignVision.getInstance().isAligned();
    }
}