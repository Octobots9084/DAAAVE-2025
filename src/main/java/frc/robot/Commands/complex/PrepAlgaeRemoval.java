package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;

public class PrepAlgaeRemoval extends Command {
    public boolean elevatorInPos = false;
    public boolean isAligned = false;
    private Elevator elevator = Elevator.getInstance();
    private ReefTargetOrientation targetOrientation;
    private ElevatorStates targetElevatorState;


    @Override
    public void initialize() {
      targetOrientation = Swerve.getInstance().getReefTargetOrientation();
      Swerve.getInstance().setDriveState(DriveState.AlignReef);
      Swerve.getInstance().setReefTargetSide(ReefTargetSide.ALGAE);
      if (targetOrientation == ReefTargetOrientation.AB || targetOrientation == ReefTargetOrientation.EF || targetOrientation == ReefTargetOrientation.IJ) {
        targetElevatorState = ElevatorStates.TOPALGAE;
      } else {
        targetElevatorState = ElevatorStates.BOTTOMALGAE;
      }
    }
    
    @Override
    public boolean isFinished() {
        if ((elevator.isAtState(targetElevatorState, 0.1)) && Swerve.getInstance().isAlignedCenterReef) {//TODO: if elevator at L4(change if theres a more precise top one) TODO: set actual tolerance
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