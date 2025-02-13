package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.AlgaeRollers.SetAlgaeRollersState;
import frc.robot.Commands.complex.CancelAllCommands;
import frc.robot.Commands.complex.ScoreCoral;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.ManualControl.ElevatorManualControl;
import frc.robot.Commands.ManualControl.WristManualControl;
import frc.robot.Commands.CoralRollers.LoadCoral;
import frc.robot.Commands.CoralRollers.OutputCoral;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.ReefSelection.SetOrientation;
import frc.robot.Commands.ReefSelection.SetTargetReefLevel;
import frc.robot.Commands.Wrist.PrepCoral;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollersStates;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Wrist.WristStates;

public class ButtonConfig {
        static CommandJoystick driverLeft = ControlMap.DRIVER_LEFT;
        static CommandJoystick driverRight = ControlMap.DRIVER_RIGHT;
        static CommandJoystick driverButtons = ControlMap.DRIVER_BUTTONS;
        static CommandJoystick coDriverLeft = ControlMap.CO_DRIVER_LEFT;
        static CommandJoystick coDriverRight = ControlMap.CO_DRIVER_RIGHT;
        static CommandJoystick coDriverButtons = ControlMap.CO_DRIVER_BUTTONS;

        public void initTeleop() {

                driverButtons
                                .button(6)
                                .onTrue(
                                                new InstantCommand(
                                                                () -> {
                                                                        Swerve.getInstance().zeroGyro();
                                                                }));
                // reef align

                coDriverLeft.button(1).onTrue(new SetOrientation(0));
                coDriverLeft.button(2).onTrue(new SetOrientation(1));

                driverButtons.button(5).onTrue(new SetCoralRollersState(CoralRollersState.INTAKING));
                driverButtons.button(6).onTrue(new SetCoralRollersState(CoralRollersState.OUTPUT));

                coDriverButtons.button(13).whileTrue(new ElevatorManualControl(() -> MathUtil.applyDeadband(
                                -ButtonConfig.coDriverRight.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND)));

                coDriverButtons.button(14).whileTrue(new WristManualControl(() -> MathUtil.applyDeadband(
                                -ButtonConfig.coDriverLeft.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND)));

                driverButtons.button(7)
                                .onTrue((new SetWristState(WristStates.FOURTYFIVE, ClosedLoopSlot.kSlot0).withTimeout(0))
                                                .andThen(new WaitCommand(1))
                                                .andThen((new SetElevatorState(ElevatorStates.LEVEL3)).withTimeout(0)));//TODO: set MAX time until timeout x2 also new WaitCommand is 1 sec.. maybe less?

                driverButtons.button(-1).whileTrue(new ScoreCoral(null, null, null).beforeStarting((new PrepCoral()).withTimeout(0))); // TODO - change button number, add parameters, set MAX time until timeout
                // NOTE - This is just for testing:
                driverButtons.button(5).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.OUTPUT));
                driverButtons.button(6).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.INTAKE));
                driverButtons.button(7).onTrue((new SetElevatorState(ElevatorStates.LEVEL1)).withTimeout(0));//TODO: set MAX time until timeout
                driverButtons.button(8).onTrue((new SetElevatorState(ElevatorStates.LEVEL2)).withTimeout(0));//TODO: set MAX time until timeout
                driverButtons.button(9).onTrue((new SetElevatorState(ElevatorStates.LEVEL3)).withTimeout(0));//TODO: set MAX time until timeout
                driverButtons.button(10).onTrue((new SetElevatorState(ElevatorStates.LEVEL4)).withTimeout(0));//TODO: set MAX time until timeout

                coDriverButtons.button(5).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.OUTPUT));
                coDriverButtons.button(6).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.INTAKE));
                coDriverButtons.button(7).onTrue((new SetWristState(WristStates.FOURTYFIVE, ClosedLoopSlot.kSlot0).withTimeout(0))
                                .andThen(new WaitCommand(1)).andThen((new SetElevatorState(ElevatorStates.LEVEL1)).withTimeout(0)));//TODO: set MAX time until timeout x2 also new WaitCommand is 1 sec.. maybe less?
                coDriverButtons.button(8).onTrue((new SetWristState(WristStates.FOURTYFIVE, ClosedLoopSlot.kSlot0).withTimeout(0))
                                .andThen(new WaitCommand(1)).andThen((new SetElevatorState(ElevatorStates.LEVEL2)).withTimeout(0)));//TODO: set MAX time until timeout x2 also new WaitCommand is 1 sec.. maybe less?
                coDriverButtons.button(9).onTrue((new SetWristState(WristStates.FOURTYFIVE, ClosedLoopSlot.kSlot0).withTimeout(0))
                                .andThen(new WaitCommand(1)).andThen((new SetElevatorState(ElevatorStates.LEVEL3)).withTimeout(0)));//TODO: set MAX time until timeout x2 also new WaitCommand is 1 sec.. maybe less?
                coDriverButtons.button(10).onTrue((new SetWristState(WristStates.VERTICAL, ClosedLoopSlot.kSlot0).withTimeout(0))
                                .andThen(new WaitCommand(1)).andThen((new SetElevatorState(ElevatorStates.LEVEL4)).withTimeout(0)));
                coDriverButtons.button(11).onTrue((new SetWristState(WristStates.LOW, ClosedLoopSlot.kSlot0)).withTimeout(0));//TODO: set max time until timeout
                coDriverButtons.button(4).whileTrue(new LoadCoral().andThen((new PrepCoral()).withTimeout(0))); //TODO: set max time until timeout
                coDriverButtons.button(1).whileTrue(new OutputCoral());

                //driverButtons.button(-1).onTrue((new SetWristState(WristStates.FOURTYFIVE, ClosedLoopSlot.kSlot0)).withTimeout(1));//test withTimeout(can delete)
                //13 TOTAL todos for setting actual value above
        }
}
