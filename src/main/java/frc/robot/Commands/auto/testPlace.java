package frc.robot.Commands.auto;

import java.nio.file.attribute.PosixFilePermission;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetLevel;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Vision.AlignVisionIO;

public class testPlace extends Command {
    public testPlace() {

    }

    @Override
    public void initialize() {
        SmartDashboard.putString("PLACRE", "PLACE");
        Swerve.getInstance().setDriveState(DriveState.AlignReef);
        AlignVisionIO.setPoleLevel(ReefTargetLevel.L1);
        AlignVisionIO.setPoleSide(ReefTargetSide.LEFT);
        AlignVisionIO.setReefOrientation(ReefTargetOrientation.KL);
    }

    @Override
    public void execute() {
        Swerve.getInstance().driveRobotRelative(
                AlignVision.getInstance().getAlignVisionIO().getAlignChassisSpeeds(AlignState.Reef));

    }

    @Override
    public boolean isFinished() {

        return AlignVision.getInstance().getAlignVisionIO().isAligned();
    }
}
