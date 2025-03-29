package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Commands.ReefSelection.manager;
import frc.robot.Subsystems.Wrist.WristStates;

public class ScoreCoral extends Command {
    public boolean elevatorInPosition = false;
    public boolean elevatorWiderInPosition = false;

    public boolean wristInPosition = false;
    public boolean wristWiderInPosition = false;
    public boolean isAligned = false;
    private Swerve swerve;
    private CoralRollers coralRollers = CoralRollers.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Wrist wrist = Wrist.getInstance();
    private ElevatorStates targetElevatorState;
    private ReefTargetSide targetSide;
    private ReefTargetOrientation targetOrientation;
    private AlignVision alignVision = AlignVision.getInstance();

    @Override
    public void initialize() {
        targetElevatorState = manager.level;
        swerve = Swerve.getInstance();
        targetSide = manager.selectedReefSide;

        swerve.setReefTargetSide(targetSide);

        swerve.setDriveState(DriveState.AlignReef);
        if (!wrist.isAtState(targetElevatorState, 0.02)) {
            Wrist.getInstance().setState(WristStates.PREP, ClosedLoopSlot.kSlot0);
        }
        alignVision.resetTolerances();
    }

    @Override
    public void execute() {
        elevatorInPosition = elevator.isAtState(targetElevatorState, 1.5);
        elevatorWiderInPosition = elevator.isAtState(targetElevatorState, 20);

        wristInPosition = wrist.isAtState(targetElevatorState, 0.02);
        wristWiderInPosition = wrist.isAtState(targetElevatorState, 0.05);

        boolean isWristPrepped = wrist.isAtState(WristStates.PREP, 0.02);
        isAligned = alignVision.isAligned();

        if (!elevatorInPosition && isWristPrepped) {
            Elevator.getInstance().setState(targetElevatorState);
        }

        // wristState1 is to stop it from constantly setting
        if (!wristInPosition && elevatorWiderInPosition && isWristPrepped) {
            wrist.setState(targetElevatorState, ClosedLoopSlot.kSlot0);// TODO do slot (remove? make actual slot? idk)
        }

        if (elevatorInPosition
                && wristWiderInPosition
                && isAligned
                && (coralRollers.getState() != CoralRollersState.OUTPUT))
            CommandScheduler.getInstance()
                    .schedule(new EjectCoral());
    }

    @Override
    public boolean isFinished() {
        if ((!coralRollers.clawBackSensorTriggered() && targetElevatorState != ElevatorStates.LEVEL1)
                || (!coralRollers.clawFrontSensorTriggered() && targetElevatorState == ElevatorStates.LEVEL1))
            return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().setDriveState(DriveState.Manual);
    }
}
