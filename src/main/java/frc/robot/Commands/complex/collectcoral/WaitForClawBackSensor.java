package frc.robot.Commands.complex.collectCoral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;

public class WaitForClawBackSensor extends Command {
    CoralRollers coralRollers;

    public WaitForClawBackSensor() {
        coralRollers = CoralRollers.getInstance();
    }

    public boolean isFinished() {
        return coralRollers.clawBackSensorTriggered();
    }

}
