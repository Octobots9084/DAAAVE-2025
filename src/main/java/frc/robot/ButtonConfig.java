package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.Climb.DeployClimb;
import frc.robot.Commands.Climb.RunClimbRollers;
import frc.robot.Commands.Climb.SetClimbState;
import frc.robot.Commands.complex.AlgaeFlickBottom;
import frc.robot.Commands.complex.AlgaeFlickTop;
import frc.robot.Commands.complex.AlignReef;
import frc.robot.Commands.complex.AlgaeInOut;
import frc.robot.Commands.complex.AlignSource;
import frc.robot.Commands.complex.BargeAlgae;
import frc.robot.Commands.complex.BargeThrow;
import frc.robot.Commands.complex.BargeAlgae;
import frc.robot.Commands.complex.CollectAlgaeStack;
import frc.robot.Commands.complex.CoralPlaceAndAlgaeReefClear;
import frc.robot.Commands.complex.CoralPlaceAndRemoveAlgaeFast;
import frc.robot.Commands.complex.EjectCoral;
import frc.robot.Commands.complex.Elephantiasis;
import frc.robot.Commands.complex.GroundAlgaeAlign;
// import frc.robot.Commands.complex.ClearAlgae;
import frc.robot.Commands.complex.Intake;
import frc.robot.Commands.complex.PrepReefPlacement;
import frc.robot.Commands.complex.RemoveAlgaeBottom;
import frc.robot.Commands.complex.RemoveAlgaeTop;
import frc.robot.Commands.complex.RobotSafeState;
import frc.robot.Commands.complex.RobotStop;
import frc.robot.Commands.complex.ScoreCoral;
import frc.robot.Commands.complex.ScoreCoralAndBackOff;
import frc.robot.Commands.complex.groundAlgae;
import frc.robot.Commands.swerve.drivebase.SetDriveState;
import frc.robot.States.ReefTargetSide;
import frc.robot.Commands.CoralRollers.SetAlgaeRollerState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.ReefSelection.ReefLevelSelection;
import frc.robot.Commands.ReefSelection.SetOrientation;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Commands.complex.ClearAlgae;
import frc.robot.Subsystems.Climb.ClimbStates;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;
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

        driverRight.button(2).whileTrue(new AlignSource().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));
        driverRight.button(2).onFalse(new InstantCommand(() -> {
            Swerve.getInstance().setDriveState(DriveState.Manual);
        }));

        // driverLeft.button(1).onTrue(new ClearAlgae());

        driverButtons.button(1)
                .onTrue(new EjectCoral());

        driverButtons.button(2).onTrue(new InstantCommand(() -> {
            Swerve.getInstance().setDriveState(DriveState.AlignReef);
        }));
        driverButtons.button(2).onFalse(new InstantCommand(() -> {
            Swerve.getInstance().setDriveState(DriveState.Manual);
        }));

        driverButtons.button(4).onTrue(new Intake().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));

        // Zero gyro button
        driverButtons.button(6).onTrue(new InstantCommand(() -> {
            Swerve.getInstance().zeroGyro();
        }));

        driverButtons.button(20).onTrue(new InstantCommand(() -> {
            Swerve.getInstance().rotLock = false;
        }));
        driverButtons.button(20).onFalse(new InstantCommand(() -> {
            Swerve.getInstance().rotLock = true;
        }));

        // driverButtons.button(2).onTrue(new RemoveAlgaeBottom().onlyIf(
        // () -> {
        // return !CoralRollers.getInstance().HasCoral();
        // }));
        // driverButtons.button(5).onTrue(new RemoveAlgaeTop().onlyIf(
        // () -> {
        // return !CoralRollers.getInstance().HasCoral();
        // }));

        // driverButtons.button(17).whileTrue(new Elephantiasis().onlyIf(
        // () -> {
        // return !CoralRollers.getInstance().HasCoral();
        // }));

        // Climb mode active (Switch 20)
        // driverButtons.button(14).and(driverButtons.button(20)).whileTrue(new
        // SetClimbState(ClimbStates.Stored));
        // driverButtons.button(16).and(driverButtons.button(20)).whileTrue(new
        // SetClimbState(ClimbStates.Deployed));
        // driverButtons.button(14).and(driverButtons.button(20)).onFalse(new
        // SetClimbState(ClimbStates.Climbing));
        // driverButtons.button(16).and(driverButtons.button(20)).onFalse(new
        // SetClimbState(ClimbStates.Climbing));
        // driverButtons.button(17).and(driverButtons.button(20)).whileTrue(new
        // ZeroClimb());

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
        coDriverButtons.button(20).onTrue(new DeployClimb()); // new ClimbSequence());
       
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
        driverLeft.button(1).whileTrue(new GroundAlgaeAlign())
                .onFalse(new SetDriveState(DriveState.Manual));
        driverButtons.button(9).onTrue(new RobotStop());
        driverButtons.button(8).onTrue(new RobotSafeState());
        driverButtons.button(7).onTrue(new groundAlgae());
        driverButtons.button(2).onTrue(new BargeThrow());
        
    }
}
