package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;

public class AlignSource extends Command {

  Swerve swerve = Swerve.getInstance();

  @Override
  public void initialize() {
    swerve.setDriveState(DriveState.AlignSource);
    CommandScheduler.getInstance().schedule(new PrepSourceCollect());
  }

  @Override
  public boolean isFinished() {
    // if the coral is inside the robot return true
    return false;
  }
}
