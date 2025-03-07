package frc.robot.Commands.complex.collectCoral;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersIO;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class WaitForClawFrontSensor extends Command {
    CoralRollers coralRollers;
    CoralRollersIO io;
    double stallStartTime = 0;

    public WaitForClawFrontSensor() {
        coralRollers = CoralRollers.getInstance();
        io = coralRollers.io;
    }

    public void initalize() {
        stallStartTime = Timer.getFPGATimestamp();
    }

    // check if the rollers are stalled (coral jam) and attempt to unjam them
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        if (!io.isStalled()) {
            stallStartTime = currentTime;
        }

        if (currentTime - stallStartTime > 0.2) {
            CoralRollers.getInstance().setState(CoralRollersState.OUTPUT);
        }
        if (!(currentTime - stallStartTime > 0.2)) {
            CoralRollers.getInstance().setState(CoralRollersState.INTAKING);
        }

    }

    public boolean isFinished() {
        return coralRollers.clawFrontSensorTriggered();
    }
}