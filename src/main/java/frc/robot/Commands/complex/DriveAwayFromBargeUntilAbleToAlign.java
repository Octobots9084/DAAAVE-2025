package frc.robot.Commands.complex;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.Swerve;

public class DriveAwayFromBargeUntilAbleToAlign extends Command{
    Swerve swerve;
    public DriveAwayFromBargeUntilAbleToAlign () {
        swerve = Swerve.getInstance();
    }

    @Override
    public void initialize() {
        if (Constants.isBlueAlliance) {
            swerve.driveFieldRelative(new ChassisSpeeds(-1, 0, 0));
        } else {
            swerve.driveFieldRelative(new ChassisSpeeds(1, 0, 0));
        }
    }


    @Override
    public boolean isFinished() {
        return swerve.ableToAlignBarge();
    }
}
