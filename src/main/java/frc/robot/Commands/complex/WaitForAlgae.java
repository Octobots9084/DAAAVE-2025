package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersIO;


public class WaitForAlgae extends Command{
    CoralRollersIO io = CoralRollers.getInstance().io;

    public boolean isFinished() {
        return io.isStalled();
    }
}