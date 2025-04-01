package frc.robot.Commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;

public class DriveBack extends Command {
    @Override
    public void execute() {
        Swerve.getInstance().driveRobotRelative(new ChassisSpeeds(-1.2, 0, 0));
    }
}
