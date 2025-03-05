package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Swerve.*;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;

public class CancelAllCommands extends InstantCommand {
    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
        Swerve.getInstance().setDriveState(DriveState.Manual);
    }

}