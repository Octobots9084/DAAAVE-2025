package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;

public class AlignReef extends Command {
  private Swerve swerve = Swerve.getInstance();
  // TODO implement this using vlaues from elevator and swerve

  @Override
  public void initialize() {
    swerve.setDriveState(DriveState.AlignReef);

    // CommandScheduler.getInstance().schedule(new PrepReefPlacement(elevatorState, wristState));
  }

  @Override
  public boolean isFinished() {
    CommandScheduler.getInstance().schedule(new SetCoralRollersState(CoralRollersState.OUTPUT));

    // check if the coral has left robot
    return !CoralRollers.getInstance().hasCoral();
  }
}
