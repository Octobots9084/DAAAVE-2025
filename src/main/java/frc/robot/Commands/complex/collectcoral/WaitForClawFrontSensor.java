package frc.robot.Commands.complex.collectcoral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;

public class WaitForClawFrontSensor extends Command {
    CoralRollers coralRollers;

    public WaitForClawFrontSensor() {
        coralRollers = CoralRollers.getInstance();
    }

    public boolean isFinished() {
        return coralRollers.clawFrontSensorTriggered();
    }

}