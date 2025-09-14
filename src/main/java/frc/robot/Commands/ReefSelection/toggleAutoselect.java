package frc.robot.Commands.ReefSelection;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveIO;

public class toggleAutoselect extends InstantCommand {

    private boolean currentState;
    private Swerve swerve = Swerve.getInstance();

  @Override
  public void initialize() {
   currentState = swerve.getInstance().getAutoselectState();
    if (currentState) {
        swerve.setAutoselectState(false);
    } else {
        swerve.setAutoselectState(true);
    }
  }
}
