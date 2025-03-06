package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.CoralRollers.SetAlgaeRollerState;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;

public class ProcessorAlign extends Command {

  Swerve swerve = Swerve.getInstance();

  @Override
  public void initialize() {
    swerve.setDriveState(DriveState.AlignProcessor);
  }

  @Override
  public boolean isFinished() {
    if (swerve.isAlignedToProcessor) {
      CommandScheduler.getInstance().schedule(new SetAlgaeRollerState(CoralRollersState.OUTPUT));
    }
    // if the algae has left the robot return true
    return false;
  }
}
