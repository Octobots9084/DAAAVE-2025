package frc.robot.Commands.complex.collectCoral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersIO;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class WaitForClawFrontSensor extends Command {
    CoralRollers coralRollers;
    CoralRollersIO io;
    boolean reversing = false;

    public WaitForClawFrontSensor() {
        coralRollers = CoralRollers.getInstance();
        io = coralRollers.io;
    }
    
    // check if the rollers are stalled (coral jam) and attempt to unjam them
    public void execute() {
        if (io.isStalled() && !reversing) {
            CoralRollers.getInstance().setState(CoralRollersState.OUTPUT);
            reversing = true;
        }
        if (!io.isStalled() && reversing) {
            CoralRollers.getInstance().setState(CoralRollersState.INTAKING);
            reversing = false;
        }
    }

    public boolean isFinished() {
        return coralRollers.clawFrontSensorTriggered();
    }

}