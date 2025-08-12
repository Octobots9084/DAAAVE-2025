package frc.robot.Commands.auto;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.ReefSelection.manager;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class testSuperCycleInAuto extends Command {
    ReefTargetOrientation targetOrientation;
    ElevatorStates targetLevel;
    ReefTargetSide targetSide;
    private Debouncer debouncer;

    public testSuperCycleInAuto(ElevatorStates targetLevel, ReefTargetSide targetSide, ReefTargetOrientation targetOrientation) {
        this.targetLevel = targetLevel;
        this.targetSide = targetSide;
        this.targetOrientation = targetOrientation;
    }//PLACE CORAL -> GRAB ALGAE
//michae decide later if elevator goes up or down during 2 piece algae GH to barge
    @Override
    public void initialize() {
        debouncer = new Debouncer(0.05);

        AlignVision.setPoleLevel(targetLevel);
        AlignVision.setPoleSide(targetSide);
        AlignVision.setReefOrientation(targetOrientation);
        manager.level = ElevatorStates.LEVEL4;
        CommandScheduler.getInstance().schedule(new PrepReefPlacementAuto());
    }

    @Override
    public void execute() {
        Swerve.getInstance().driveRobotRelative(AlignVision.getInstance().getAlignChassisSpeeds(AlignState.Reef));
    }
//TODO:supercycle make this + remove algae in auto in super cycle into sequential command?
    @Override
    public boolean isFinished() {
        if (Constants.currentMode == Mode.SIM) {
            return true;
        }
        if (debouncer.calculate(AlignVision.getInstance().isAligned() && Elevator.getInstance().isAtState(ElevatorStates.LEVEL4, 1.5)
                && Wrist.getInstance().isAtState(ElevatorStates.LEVEL4, 0.01))) {
            CoralRollers.getInstance().setState(CoralRollersState.OUTPUT);

            //ALGAE
            new RemoveAlgaeInAutoInSuperCycle(targetOrientation, targetSide);
        }
        return CoralRollers.getInstance().isStalled() && !CoralRollers.getInstance().HasCoral();
    }


    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance()
                .schedule(new SetWristStateTolerance(WristStates.PREP, 0.01, ClosedLoopSlot.kSlot0));
        Swerve.getInstance().driveRobotRelative(new ChassisSpeeds());
    }
}