package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.*;
import frc.robot.Subsystems.Swerve.Swerve;

public class SetTargetReefOrientation extends Command {
  public SetTargetReefOrientation(ReefTargetOrientation targetOrientation) {
    Swerve.getInstance().setReefTargetOrientation(targetOrientation);
  }
}
