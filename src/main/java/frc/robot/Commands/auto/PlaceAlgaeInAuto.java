package frc.robot.Commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;

public class PlaceAlgaeInAuto extends Command {
    ReefTargetOrientation targetOrientation;

    public PlaceAlgaeInAuto(ReefTargetOrientation targetOrientation) {
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
        return CoralRollers.getInstance().isStalled();
    }
}