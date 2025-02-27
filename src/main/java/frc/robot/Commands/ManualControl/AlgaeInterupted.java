package frc.robot.Commands.ManualControl;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;

public class AlgaeInterupted extends InstantCommand{
    public void initialize(){
        Swerve.getInstance().setDriveState(DriveState.Manual);
    }
}
