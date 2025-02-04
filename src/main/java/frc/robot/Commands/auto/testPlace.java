package frc.robot.Commands.auto;

import java.nio.file.attribute.PosixFilePermission;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetLevel;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;

public class testPlace extends Command {
    ReefTargetLevel targetLevel;
    ReefTargetSide targetSide;
    ReefTargetOrientation targetOrientation;
    public testPlace(ReefTargetLevel targetLevel, ReefTargetSide targetSide, ReefTargetOrientation targetOrientation) {
        this.targetLevel = targetLevel;
        this.targetSide = targetSide;
        this.targetOrientation = targetOrientation;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("PLACRE", "PLACE");
        AlignVision.setPoleLevel(targetLevel);
        AlignVision.setPoleSide(targetSide);
        AlignVision.setReefOrientation(targetOrientation);
    }

    @Override
    public void execute() {
        Swerve.getInstance().driveRobotRelative(AlignVision.getInstance().getAlignChassisSpeeds(AlignState.Reef));

    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("is aligned", AlignVision.getInstance().isAligned());
        return AlignVision.getInstance().isAligned();
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().driveRobotRelative(new ChassisSpeeds());
    }

    
}
