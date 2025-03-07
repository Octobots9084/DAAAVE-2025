package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.States.ReefTargetLevel;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Commands.Climb.SetClimbState;
import frc.robot.Commands.Climb.ZeroClimb;
import frc.robot.Commands.complex.AlignSource;
import frc.robot.Commands.complex.BargeAlgae;
import frc.robot.Commands.complex.CancelAllCommands;
import frc.robot.Commands.complex.ClearAlgae;
import frc.robot.Commands.complex.CollectAlgaeStack;
import frc.robot.Commands.complex.EjectCoral;
import frc.robot.Commands.complex.Elephantiasis;
// import frc.robot.Commands.complex.ClearAlgae;
import frc.robot.Commands.complex.Intake;
import frc.robot.Commands.complex.PrepReefPlacement;
import frc.robot.Commands.complex.RemoveAlgaeBottom;
import frc.robot.Commands.complex.RemoveAlgaeTop;
import frc.robot.Commands.complex.RobotSafeState;
import frc.robot.Commands.complex.RobotStop;
import frc.robot.Commands.complex.ScoreCoral;
import frc.robot.Commands.complex.collectCoral.WaitForCoralDetected;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
// import frc.robot.Commands.ManualControl.AlgaeInterupted;
import frc.robot.Commands.ManualControl.WristManualControl;
import frc.robot.Commands.CoralRollers.LoadCoral;
import frc.robot.Commands.CoralRollers.OutputCoral;
import frc.robot.Commands.CoralRollers.ReverseRollersWileTrue;
import frc.robot.Commands.CoralRollers.SetAlgaeRollerState;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.ReefSelection.ReefLevelSelection;
import frc.robot.Commands.ReefSelection.SetOrientation;
import frc.robot.Commands.Wrist.PrepCoral;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Commands.auto.AlignCollect;
import frc.robot.Subsystems.Climb.ClimbStates;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
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
        driverRight.button(1).whileTrue(new ScoreCoral().onlyIf(
                () -> {
                    return CoralRollers.getInstance().HasCoral();
                }));
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

        driverButtons.button(2).onTrue(new SetAlgaeRollerState(CoralRollersState.AlGAEOUTPUT).onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }).withTimeout(1.5));

        driverButtons.button(4).onTrue(new Intake().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));

        // Zero gyro button
        driverButtons.button(6).onTrue(new InstantCommand(() -> {
            Swerve.getInstance().zeroGyro();
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

        coDriverButtons.button(1).onTrue(new EjectCoral());
        coDriverButtons.button(4).onTrue(new Intake().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));
        coDriverButtons.button(2).onTrue(new RemoveAlgaeTop().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));
        coDriverButtons.button(5).onTrue(new RemoveAlgaeBottom().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));
        coDriverButtons.button(8).onTrue(new CollectAlgaeStack());
        coDriverButtons.button(3).onTrue(new BargeAlgae());

        coDriverButtons.button(6).onTrue(new PrepReefPlacement());

        // Return robot to a safe configuration
        // Score and Intake assistance buttons for right stick
        coDriverButtons.button(7).onTrue(new RobotSafeState());

        coDriverButtons.button(9).onTrue(new RobotStop());

        // Reef mode active (Switch 20)
        // Reef selection
        coDriverButtons.button(20).onTrue(new SetWristState(WristStates.L1, ClosedLoopSlot.kSlot0));
        coDriverButtons.button(10).and(coDriverButtons.button(20).negate()).onTrue(new ReefLevelSelection(4));
        coDriverButtons.button(12).and(coDriverButtons.button(20).negate()).onTrue(new ReefLevelSelection(3));
        coDriverButtons.button(14).and(coDriverButtons.button(20).negate()).onTrue(new ReefLevelSelection(2));
        coDriverButtons.button(16).and(coDriverButtons.button(20).negate()).onTrue(new ReefLevelSelection(1));
        coDriverButtons.button(17).and(coDriverButtons.button(20).negate()).whileTrue(new Elephantiasis().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));
        coDriverRight.button(1).onTrue(new SetOrientation(0));
        coDriverRight.button(2).onTrue(new SetOrientation(1));
        // Climb mode active (Switch 20)
        coDriverButtons.button(14).and(coDriverButtons.button(20))
                .whileTrue(new SetWristState(WristStates.L1, ClosedLoopSlot.kSlot0).andThen(new SetClimbState(ClimbStates.Stored)));
        coDriverButtons.button(16).and(coDriverButtons.button(20))
                .whileTrue(new SetWristState(WristStates.L1, ClosedLoopSlot.kSlot0).andThen(new SetClimbState(ClimbStates.Deployed)));
        coDriverButtons.button(14).and(coDriverButtons.button(20))
                .onFalse(new SetClimbState(ClimbStates.Climbing));
        coDriverButtons.button(16).and(coDriverButtons.button(20)).onFalse(new SetClimbState(ClimbStates.Climbing));
        coDriverButtons.button(17).and(coDriverButtons.button(20)).whileTrue(new ZeroClimb());

    }
}
