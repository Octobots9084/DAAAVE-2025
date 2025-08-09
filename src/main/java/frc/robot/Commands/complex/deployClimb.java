package frc.robot.Commands.complex;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.States.ReefTargetSide;
import frc.robot.Commands.Climb.RunClimbRollers;
import frc.robot.Commands.Climb.SetClimbState;
import frc.robot.Commands.Climb.StopClimbTemporary;
import frc.robot.Commands.ReefSelection.ReefLevelSelection;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbStates;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.WristStates;

public class deployClimb extends SequentialCommandGroup{
    public deployClimb(){
        addCommands(
            new SetWristState(WristStates.ALGAEREMOVAL, ClosedLoopSlot.kSlot0),
            new RunClimbRollers(), 
            new SetClimbState(ClimbStates.Deployed, ClosedLoopSlot.kSlot1),
            new WaitUntilCommand(() -> Climb.getInstance().isClimbClamped()),
            new WaitCommand(0.5),
            new SetClimbState(ClimbStates.Climbing, ClosedLoopSlot.kSlot0),
            new StopClimbTemporary()
        ); 
        }
                
    }

//DIGITAL INPUTS -> 7, 9