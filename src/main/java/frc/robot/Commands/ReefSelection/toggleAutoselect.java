package frc.robot.Commands.ReefSelection;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Swerve;

public class toggleAutoselect extends InstantCommand {

    private boolean currentState;

  @Override
  public void initialize() {
   currentState = Swerve.getAutoselectState();
    if (currentState) {
        Swerve.setAutoselectState(false);
    } else {
        Swerve.setAutoselectState(true);
    }
  }
}
