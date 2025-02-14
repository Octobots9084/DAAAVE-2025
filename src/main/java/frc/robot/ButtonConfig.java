package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.AlgaeRollers.SetAlgaeRollersState;
import frc.robot.Commands.complex.PrepIntake;
import frc.robot.Commands.complex.ScoreCoral;
import frc.robot.Commands.complex.collectCoral.WaitForCoralDetected;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.ManualControl.ElevatorManualControl;
import frc.robot.Commands.ManualControl.WristManualControl;
import frc.robot.Commands.CoralRollers.LoadCoral;
import frc.robot.Commands.CoralRollers.OutputCoral;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.ReefSelection.ReefLevelSelection;
import frc.robot.Commands.ReefSelection.SetOrientation;
import frc.robot.Commands.Wrist.PrepCoral;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollersStates;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
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

                driverButtons.button(6).onTrue(new InstantCommand(() -> {
                        Swerve.getInstance().zeroGyro();
                }));
                // reef align

                // reef selection
                coDriverRight.button(1).onTrue(new SetOrientation(0));
                coDriverRight.button(2).onTrue(new SetOrientation(1));

                // level selection
                coDriverButtons.button(10).onTrue(new ReefLevelSelection(4));
                coDriverButtons.button(12).onTrue(new ReefLevelSelection(3));
                coDriverButtons.button(14).onTrue(new ReefLevelSelection(2));
                coDriverButtons.button(16).onTrue(new ReefLevelSelection(1));

                // driverButtons.button(5).onTrue(new
                // SetCoralRollersState(CoralRollersState.INTAKING));
                // driverButtons.button(6).onTrue(new
                // SetCoralRollersState(CoralRollersState.OUTPUT));

                // coDriverButtons.button(13).whileTrue(new ElevatorManualControl(() ->
                // MathUtil.applyDeadband(
                // -ButtonConfig.coDriverRight.getRawAxis(1),
                // OperatorConstants.LEFT_Y_DEADBAND)));

                // coDriverButtons.button(14).whileTrue(new WristManualControl(() ->
                // MathUtil.applyDeadband(
                // -ButtonConfig.coDriverLeft.getRawAxis(0),
                // OperatorConstants.LEFT_X_DEADBAND)));

                // driverButtons.button(7)
                // .onTrue((new SetWristState(WristStates.FOURTYFIVE, ClosedLoopSlot.kSlot0)
                // .withTimeout(0))
                // .andThen(new WaitCommand(1))
                // .andThen((new SetElevatorState(ElevatorStates.LEVEL3)).withTimeout(0)));//
                // TODO:
                // // set
                // // MAX
                // time
                // until
                // timeout
                // x2
                // also
                // new
                // WaitCommand
                // is
                // 1
                // sec..
                // maybe
                // less?

                // driverButtons.button(-1).whileTrue(new ScoreCoral(null, null, null)
                // .beforeStarting((new PrepCoral()).withTimeout(0)
                // .andThen(new SetCoralRollersState(CoralRollersState.STOPPED))
                // .andThen((new SetWristState(WristStates.PREP, ClosedLoopSlot.kSlot0))// TODO
                // // do
                // // kslot
                // // (remove/change
                // // to
                // // correct
                // // val)
                // .withTimeout(0))// TODO set timeout
                // .andThen((new SetElevatorState(ElevatorStates.LOW))// TODO set elevator
                // // state
                // .withTimeout(0))// TODO set timeout
                // )); // TODO - change button number, add parameters, set MAX time until
                // timeout

                // driverButtons.button(5).onTrue(new
                // SetAlgaeRollersState(AlgaeRollersStates.OUTPUT));
                // driverButtons.button(6).onTrue(new
                // SetAlgaeRollersState(AlgaeRollersStates.INTAKE));
                // driverButtons.button(7).onTrue((new
                // SetElevatorState(ElevatorStates.LEVEL1)).withTimeout(0));//TODO: set MAX time
                // until timeout
                // driverButtons.button(8).onTrue((new
                // SetElevatorState(ElevatorStates.LEVEL2)).withTimeout(0));//TODO: set MAX time
                // until timeout
                // driverButtons.button(9).onTrue((new
                // SetElevatorState(ElevatorStates.LEVEL3)).withTimeout(0));//TODO: set MAX time
                // until timeout
                // driverButtons.button(10).onTrue((new
                // SetElevatorState(ElevatorStates.LEVEL4)).withTimeout(0));//TODO: set MAX time
                // until timeout

                coDriverButtons.button(5).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.OUTPUT));
                coDriverButtons.button(6).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.INTAKE));
                coDriverButtons.button(7).onTrue(
                                (new SetWristState(WristStates.L1, ClosedLoopSlot.kSlot0).withTimeout(0))
                                                .andThen(new WaitCommand(1))
                                                .andThen((new SetElevatorState(ElevatorStates.LEVEL1)).withTimeout(0)));// TODO:
                                                                                                                        // set
                                                                                                                        // MAX
                                                                                                                        // time
                                                                                                                        // until
                                                                                                                        // timeout
                                                                                                                        // x2
                                                                                                                        // also
                                                                                                                        // new
                                                                                                                        // WaitCommand
                                                                                                                        // is
                                                                                                                        // 1
                                                                                                                        // sec..
                                                                                                                        // maybe
                                                                                                                        // less?
                coDriverButtons.button(8).onTrue(
                                (new SetWristState(WristStates.L2,
                                                ClosedLoopSlot.kSlot0).withTimeout(0))
                                                .andThen(new WaitCommand(1))
                                                .andThen((new SetElevatorState(ElevatorStates.LEVEL2)).withTimeout(0)));//
                // TODO:
                // set
                // MAX
                // time
                // until
                // timeout
                // x2
                // also
                // new
                // WaitCommand
                // is
                // 1
                // sec..
                // maybe
                // less?
                coDriverButtons.button(9).onTrue(
                                (new SetWristState(WristStates.L3, ClosedLoopSlot.kSlot0).withTimeout(0))
                                                .andThen(new WaitCommand(1))
                                                .andThen((new SetElevatorState(ElevatorStates.LEVEL3)).withTimeout(0)));// TODO:
                                                                                                                        // set
                                                                                                                        // MAX
                                                                                                                        // time
                                                                                                                        // until
                                                                                                                        // timeout
                                                                                                                        // x2
                                                                                                                        // also
                                                                                                                        // new
                                                                                                                        // WaitCommand
                                                                                                                        // is
                                                                                                                        // 1
                                                                                                                        // sec..
                                                                                                                        // maybe
                                                                                                                        // less?
                coDriverButtons.button(10)
                                .onTrue((new SetWristState(WristStates.L4, ClosedLoopSlot.kSlot0).withTimeout(0))
                                                .andThen(new WaitCommand(1))
                                                .andThen((new SetElevatorState(ElevatorStates.LEVEL4)).withTimeout(0)));
                coDriverButtons.button(11)
                                .onTrue(new SetElevatorStateTolerance(ElevatorStates.LEVEL1, 1.5)
                                                .andThen(new SetWristState(WristStates.INTAKE,
                                                                ClosedLoopSlot.kSlot0)));// TODO:
                // set
                // max
                // time
                // until
                // timeout
                coDriverButtons.button(4)
                                .whileTrue(new LoadCoral().beforeStarting(new SetElevatorState(ElevatorStates.LOW))
                                                .andThen((new PrepCoral()).withTimeout(0))); // TODO: set max time until
                                                                                             // timeout
                coDriverButtons.button(1).whileTrue(new InstantCommand(() -> {
                    CoralRollers.getInstance().setState(CoralRollersState.INTAKING);
                }).andThen(new WaitForCoralDetected()).andThen(new WaitCommand(0.5)).andThen(new InstantCommand(() -> {
                    CoralRollers.getInstance().setState(CoralRollersState.STOPPED);
                })));
                coDriverButtons.button(2).whileTrue(new InstantCommand(() -> {
                        CoralRollers.getInstance().setState(CoralRollersState.OUTPUT);
                }));
                coDriverButtons.button(3).whileTrue(new InstantCommand(() -> {
                        CoralRollers.getInstance().setState(CoralRollersState.STOPPED);
                }));

                // driverButtons.button(-1).onTrue((new SetWristState(WristStates.FOURTYFIVE,
                // ClosedLoopSlot.kSlot0)).withTimeout(1));//test withTimeout(can delete)
                // 13 TOTAL todos for setting actual value above
        }
}
