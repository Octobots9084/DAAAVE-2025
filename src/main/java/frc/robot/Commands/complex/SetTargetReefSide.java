package frc.robot.Commands.Complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.*;
import frc.robot.Subsystems.Swerve.Swerve;

public class SetTargetReefSide extends Command {
  public SetTargetReefSide(ReefTargetSide targetSide) {
    Swerve.getInstance().setReefTargetSide(targetSide);
  }
}
