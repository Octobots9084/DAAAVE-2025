package frc.robot;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.Climb.SetClimbState;
import frc.robot.Commands.complex.AlgaeFlickBottom;
import frc.robot.Commands.complex.AlgaeFlickTop;
import frc.robot.Commands.complex.AlignSource;
import frc.robot.Commands.complex.BargeAlgae;
import frc.robot.Commands.complex.BargeThrow;
import frc.robot.Commands.complex.CollectAlgaeStack;
import frc.robot.Commands.complex.CoralPlaceAndRemoveAlgaeFast;
import frc.robot.Commands.complex.EjectCoral;
import frc.robot.Commands.complex.GroundAlgaeAlign;
import frc.robot.Commands.complex.Intake;
import frc.robot.Commands.complex.PrepReefPlacement;
import frc.robot.Commands.complex.RemoveAlgaeBottom;
import frc.robot.Commands.complex.RemoveAlgaeTop;
import frc.robot.Commands.complex.RobotSafeState;
import frc.robot.Commands.complex.RobotStop;
import frc.robot.Commands.complex.ScoreCoralAndBackOff;
import frc.robot.Commands.complex.deployClimbAuto;
import frc.robot.Commands.complex.groundAlgae;
import frc.robot.Commands.swerve.drivebase.SetDriveState;
import frc.robot.States.ReefTargetSide;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.ReefSelection.ReefLevelSelection;
import frc.robot.Commands.ReefSelection.SetOrientation;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Commands.complex.ClearAlgae;
import frc.robot.Subsystems.Climb.ClimbStates;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class ButtonConfig {
    static CommandJoystick driverLeft = ControlMap.DRIVER_LEFT;
    static CommandJoystick driverRight = ControlMap.DRIVER_RIGHT;
    static CommandJoystick driverButtons = ControlMap.DRIVER_BUTTONS;
    static CommandJoystick coDriverLeft = ControlMap.CO_DRIVER_LEFT;
    static CommandJoystick coDriverRight = ControlMap.CO_DRIVER_RIGHT;
    static CommandJoystick coDriverButtons = ControlMap.CO_DRIVER_BUTTONS;

    public void initTeleop() {
        // Score and Intake assistance buttons for right stick
        driverRight.button(1).whileTrue(new ScoreCoralAndBackOff().onlyIf(
                () -> {
                    return CoralRollers.getInstance().HasCoral();
                })).onFalse(new SetDriveState(DriveState.Manual));
        
        // Alligns to the sorse if it has coral for right stick
        driverRight.button(2).whileTrue(new AlignSource().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));
        driverRight.button(2).onFalse(new InstantCommand(() -> {
            Swerve.getInstance().setDriveState(DriveState.Manual);
        }));

        // Ejects coral in driver side
        driverButtons.button(1)
                .onTrue(new EjectCoral());

        // Just alligns to the middle of the selected reef side
        driverButtons.button(2).onTrue(new InstantCommand(() -> {
            Swerve.getInstance().setDriveState(DriveState.AlignReef);
        })).onTrue( new InstantCommand(() -> {
            Swerve.getInstance().setReefTargetSide(ReefTargetSide.ALGAE);
        }));

        //sets drive state back to manuel from allighn button
        driverButtons.button(2).onFalse(new InstantCommand(() -> {
            Swerve.getInstance().setDriveState(DriveState.Manual);
        }));

        // Intake for driver
        driverButtons.button(4).onTrue(new Intake().onlyIf(
                () -> {
                    return (!CoralRollers.getInstance().HasCoral() || Wrist.getInstance().isAtState(WristStates.INTAKE,0.01));
                }));

        // Zero gyro button
        driverButtons.button(6).onTrue(new InstantCommand(() -> {
            Swerve.getInstance().zeroGyro();
        }));

        // WTF
        driverButtons.button(20).onTrue(new InstantCommand(() -> {
            Swerve.getInstance().rotLock = false;
        }));

        // WTF
        driverButtons.button(20).onTrue(new InstantCommand(() -> {
            
        }));

        coDriverButtons.button(1).onTrue(new EjectCoral().andThen(new RobotStop()));
        coDriverButtons.button(4).onTrue(new Intake().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));
        coDriverButtons.button(2).onTrue(new ConditionalCommand(new AlgaeFlickTop(), new RemoveAlgaeTop().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }), coDriverButtons.button(19)));
        coDriverButtons.button(5).onTrue(new ConditionalCommand(new AlgaeFlickBottom(), new RemoveAlgaeBottom().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }), coDriverButtons.button(19)));

        coDriverButtons.button(8).onTrue(new CollectAlgaeStack());
        coDriverButtons.button(3).onTrue(new BargeAlgae());

        coDriverButtons.button(6).onTrue(new PrepReefPlacement());
        coDriverButtons.button(15).onTrue(new SetElevatorState(ElevatorStates.LEVEL4).andThen(new SetWristStateTolerance(WristStates.BARGEALGAE, 1, ClosedLoopSlot.kSlot0)));

        // Return robot to a safe configuration
        // Score and Intake assistance buttons for right stick
        coDriverButtons.button(7).onTrue(new RobotSafeState());

        coDriverButtons.button(9).onTrue(new RobotStop());

        // Reef mode active (Switch 20)
        // Reef selection
        coDriverButtons.button(20).onTrue(new deployClimbAuto()); // new ClimbSequence());
       
        coDriverButtons.button(10).onTrue(new ReefLevelSelection(4));
        coDriverButtons.button(12).onTrue(new ReefLevelSelection(3));
        coDriverButtons.button(14).onTrue(new ReefLevelSelection(2));
        coDriverButtons.button(16)
                .onTrue(new ConditionalCommand(new InstantCommand(), new ReefLevelSelection(1).andThen(new InstantCommand(() -> {
                    AlignVision.setPoleSide(ReefTargetSide.ALGAE);
                })), coDriverButtons.button(20)));
       
       //coDriverButtons.button(16).onTrue(new ConditionalCommand(new SetClimbState(ClimbStates.Deployed, ClosedLoopSlot.kSlot1), new InstantCommand(), coDriverButtons.button(20)));
       
      

       // coDriverButtons.button(16).onFalse(new ConditionalCommand(new
        // SetClimbState(ClimbStates.Stored), new InstantCommand(),
        // coDriverButtons.button(20)));

        coDriverButtons.button(17).onTrue(new ConditionalCommand(new SetClimbState(ClimbStates.Climbing, ClosedLoopSlot.kSlot0), new InstantCommand(), coDriverButtons.button(20)));
        // coDriverButtons.button(17).onFalse(new ConditionalCommand(new
        // SetClimbState(ClimbStates.Stored), new InstantCommand(),
        // coDriverButtons.button(20)));

        coDriverRight.button(1).onTrue(new SetOrientation(0));
        coDriverRight.button(2).onTrue(new SetOrientation(1));
        coDriverButtons.button(13).onTrue(new SetWristStateTolerance(WristStates.PREP, 0.01, ClosedLoopSlot.kSlot0)
                .andThen(new SetElevatorStateTolerance(ElevatorStates.CLIMB, 1.5).andThen(new SetWristState(WristStates.TUNING, ClosedLoopSlot.kSlot0))));
        // Climb mode active (Switch 20)


        driverLeft.button(2).whileTrue(new ConditionalCommand(new CoralPlaceAndRemoveAlgaeFast(), new ClearAlgae(), () -> CoralRollers.getInstance().HasCoral()))
                .onFalse(new SetDriveState(DriveState.Manual));
        driverButtons.button(9).onTrue(new RobotStop());
        driverButtons.button(8).onTrue(new RobotSafeState());
        driverButtons.button(7).onTrue(new groundAlgae());
        driverButtons.button(11).whileTrue(new GroundAlgaeAlign()).onFalse(new InstantCommand(() -> {
            Swerve.getInstance().setDriveState(DriveState.Manual);
        }));
        driverLeft.button(1).onTrue(new BargeThrow());
        
    }
}
