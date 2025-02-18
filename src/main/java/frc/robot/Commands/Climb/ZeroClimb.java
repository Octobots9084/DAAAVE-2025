package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb.Climb;

public class ZeroClimb extends Command {
    @Override
    public void initialize() {
        Climb.getInstance().setVoltage(9);
    }

    @Override
    public void execute() {
        Climb.getInstance().zeroEncoder();

    }

    @Override
    public void end(boolean interrupted) {
        Climb.getInstance().setVoltage(0);

    }

}
