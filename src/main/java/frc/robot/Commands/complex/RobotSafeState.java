package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class RobotSafeState extends InstantCommand {
    Elevator elevator;
    Wrist wrist;
    CoralRollers coralRollers;
    Swerve swerve;

    public RobotSafeState() {
        elevator = Elevator.getInstance();
        wrist = Wrist.getInstance();
        coralRollers = CoralRollers.getInstance();
        swerve = Swerve.getInstance();
    }

    @Override
    public void initialize() {
        elevator.setState(ElevatorStates.LEVEL1);
        wrist.setState(WristStates.PREP, ClosedLoopSlot.kSlot0);
        coralRollers.setState(CoralRollersState.STOPPED);
        swerve.setDriveState(DriveState.Manual);

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
    }
}
