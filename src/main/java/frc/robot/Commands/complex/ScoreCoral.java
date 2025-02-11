package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.ReefSelection.SetTargetReefLevel;
import frc.robot.Commands.ReefSelection.SetTargetReefOrientation;
import frc.robot.Commands.ReefSelection.SetTargetReefSide;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Wrist.Wrist;

public class ScoreCoral extends Command {
  public boolean elevatorInPos = false;
  public boolean wristInPosition = false;
  public boolean isAligned = false;
  private Swerve swerve;
  private CoralRollers coralRollers = CoralRollers.getInstance();
  private Elevator elevator = Elevator.getInstance();
  private Wrist wrist = Wrist.getInstance();
  private ElevatorStates targetElevatorState;
  private ReefTargetSide targetSide;
  private ReefTargetOrientation targetOrientation;

  public ScoreCoral(ElevatorStates elevatorState, ReefTargetSide side, ReefTargetOrientation orientation) {
    targetElevatorState = elevatorState;
    targetSide = side;
    targetOrientation = orientation;
    swerve = Swerve.getInstance();
  }

  @Override
  public void initialize() {
    
    if (coralRollers.io.HasCoral()) {
        swerve.setDriveState(DriveState.AlignReef);
        CommandScheduler.getInstance().schedule(new SetTargetReefLevel(targetElevatorState));
        CommandScheduler.getInstance().schedule(new SetTargetReefSide(targetSide));
        CommandScheduler.getInstance().schedule(new SetTargetReefOrientation(targetOrientation));
    }
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("place L4", true);
    elevatorInPos = elevator.isAtState(targetElevatorState, 0.1 /*TODO - set actual tolerance*/);
    wristInPosition = wrist.isAtState(targetElevatorState, 0.1 /*TODO - set actual tolerance*/);
    isAligned = true; // Need vision code

    if (elevatorInPos
            && wristInPosition
            && isAligned
            && (coralRollers.getState() != CoralRollersState.OUTPUT))
        CommandScheduler.getInstance()
            .schedule(new SetCoralRollersState(CoralRollersState.OUTPUT));
  }

    @Override
    public boolean isFinished() {
        if (coralRollers.io.IsIntaking() == false)
            return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance()
            .schedule(new SetCoralRollersState(CoralRollersState.STOPPED));
        //swerve.setDriveState(DriveState.Manual);
        CommandScheduler.getInstance()
              .schedule(new PrepIntake(false));
    }
  // new AlignReef().andThen(new SetCoralRollersState(CoralRollersState.REJECTING))
}
