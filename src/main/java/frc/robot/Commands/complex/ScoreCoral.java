package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

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
import frc.robot.Subsystems.Wrist.WristStates;

public class ScoreCoral extends Command {
  public boolean elevatorInPosition = false;
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
        CommandScheduler.getInstance().schedule(new SetTargetReefSide(targetSide));
        CommandScheduler.getInstance().schedule(new SetTargetReefOrientation(targetOrientation));
    }
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("place L4", true);
    elevatorInPosition = elevator.isAtState(targetElevatorState, 0.1 /*TODO - set actual tolerance*/);
    wristInPosition = wrist.isAtState(targetElevatorState, 0.1 /*TODO - set actual tolerance*/);
    isAligned = true; //TODO Need vision code

    if (isAligned && !elevatorInPosition) {
      Elevator.getInstance().setState(targetElevatorState);
    }

    //wristState1 is to stop it from constantly setting 
    if (isAligned && !wristInPosition) {
      Wrist.getInstance().setState(targetElevatorState, ClosedLoopSlot.kSlot0);//TODO do slot (remove? make actual slot? idk)
    }
    
    
    if (elevatorInPosition
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
        Swerve.getInstance().setDriveState(DriveState.Manual);        
    }
  // new AlignReef().andThen(new SetCoralRollersState(CoralRollersState.REJECTING))
}
