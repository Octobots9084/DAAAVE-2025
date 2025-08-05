package frc.robot.Commands.auto;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetLevel;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Commands.ReefSelection.manager;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;

public class testAlignInAuto extends Command {
    ReefTargetOrientation targetOrientation;
    ReefTargetSide targetSide;
    boolean alignAlgae;
    private Debouncer debouncer;

    public testAlignInAuto(ReefTargetSide targetSide, ReefTargetOrientation targetOrientation, boolean alignAlgae) {
        this.targetSide = targetSide;
        this.targetOrientation = targetOrientation;
        this.alignAlgae = alignAlgae;
    }
//TODO CHANGE CONSTRUCTOR BOOL -> TARGET
//make make sense
    @Override
    public void initialize() {
        debouncer = new Debouncer(0.05);

        AlignVision.setPoleLevel(ElevatorStates.LEVEL2);
        if (!alignAlgae) {
            AlignVision.setPoleSide(targetSide);
        } else {
            AlignVision.setPoleSide(ReefTargetSide.ALGAE);
        }
        AlignVision.setReefOrientation(targetOrientation);
        
        manager.level = ElevatorStates.LEVEL2;
        CommandScheduler.getInstance().schedule(new PrepReefPlacementAuto());
    }
    /*
     
        AlignVision.setPoleLevel(targetLevel);
        AlignVision.setPoleSide(targetSide);
        AlignVision.setReefOrientation(targetOrientation);
        manager.level = ElevatorStates.LEVEL4;
        CommandScheduler.getInstance().schedule(new PrepReefPlacementAuto());
     */

    @Override
    public void execute() {

        Swerve.getInstance().driveRobotRelative(AlignVision.getInstance().getAlignChassisSpeeds(AlignState.Reef));
    }

    @Override
    public boolean isFinished() {
        if (Constants.currentMode == Mode.SIM) {
            return true;
        }

        return debouncer.calculate(AlignVision.getInstance().isAligned());
    }


    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().driveRobotRelative(new ChassisSpeeds());
    }
}