package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;

public class IntakeCoral extends Command {
    private CoralRollers coralrollers = CoralRollers.getInstance();


    public IntakeCoral() {

    }

    @Override
    public void initialize() {
        coralrollers.setState(CoralRollersState.INTAKING);
    }
    
    @Override
    public void execute() {
        if (coralrollers.clawFrontSensorTriggered()) {
            coralrollers.setState(CoralRollersState.HALFINTAKE);
        }
    }

    @Override
    public boolean isFinished() {
        return coralrollers.clawBackSensorTriggered();
    }

    @Override
    public void end(boolean interrupted) {
        coralrollers.setState(CoralRollersState.STOPPED);
    }
}
