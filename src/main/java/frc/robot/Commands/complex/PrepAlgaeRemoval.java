package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class PrepAlgaeRemoval extends Command {
    public boolean elevatorInPos = false;
    public boolean isAligned = false;
    private Elevator elevator = Elevator.getInstance();
    private ReefTargetOrientation targetOrientation;
    private ElevatorStates targetElevatorState;
    private WristStates targetWristState;


    @Override
    public void initialize() {
      targetOrientation = Swerve.getInstance().getReefTargetOrientation();
      Swerve.getInstance().setReefTargetSide(ReefTargetSide.ALGAE);
      CommandScheduler.getInstance().schedule(new AlignReef());
      if (targetOrientation == ReefTargetOrientation.AB || targetOrientation == ReefTargetOrientation.EF || targetOrientation == ReefTargetOrientation.IJ) {
        targetElevatorState = ElevatorStates.TOPALGAE;
        targetWristState = WristStates.ALAGEREMOVAL;
      } else {
        targetElevatorState = ElevatorStates.BOTTOMALGAE;
        targetWristState = WristStates.ALAGEREMOVAL;
      }
      elevator.setState(targetElevatorState);
      Wrist.getInstance().setState(targetWristState,ClosedLoopSlot.kSlot0);
    }
    
    @Override
    public boolean isFinished() {
        if ((elevator.isAtState(targetElevatorState, 0.1)) && Wrist.getInstance().isAtState(targetWristState,0.1)&& Swerve.getInstance().isAlignedCenterReef) {//TODO: if elevator at L4(change if theres a more precise top one) TODO: set actual tolerance
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end (boolean interrupted) {
      Swerve.getInstance().setDriveState(DriveState.Manual);
    }

}