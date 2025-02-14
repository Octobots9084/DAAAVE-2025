package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.AlgaeRollers.SetAlgaeRollersState;
import frc.robot.Commands.complex.PrepReefPlacement;
import frc.robot.Commands.complex.ScoreCoral;
import frc.robot.Commands.complex.collectCoral.WaitForCoralDetected;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
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
import frc.robot.Subsystems.Elevator.Elevator;
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
                coDriverButtons.button(8).onTrue(new ReefLevelSelection(4));
                coDriverButtons.button(10).onTrue(new ReefLevelSelection(3));
                coDriverButtons.button(12).onTrue(new ReefLevelSelection(2));
                coDriverButtons.button(7).onTrue(new ReefLevelSelection(1));

                driverButtons.button(4)
                                .onTrue(new PrepReefPlacement());
                coDriverButtons.button(1).onTrue(new InstantCommand(() -> {
                        CoralRollers.getInstance().setState(CoralRollersState.INTAKING);
                }).andThen(new WaitForCoralDetected()).andThen(new WaitCommand(0.1)).andThen(new InstantCommand(() -> {
                        CoralRollers.getInstance().setState(CoralRollersState.STOPPED);
                })));
                coDriverButtons.button(2).whileTrue(new InstantCommand(() -> {
                        CoralRollers.getInstance().setState(CoralRollersState.OUTPUT);
                }));
                coDriverButtons.button(3).whileTrue(new InstantCommand(() -> {
                        CoralRollers.getInstance().setState(CoralRollersState.STOPPED);
                }));

                coDriverButtons.button(4).whileTrue(new InstantCommand(() -> {
                        CoralRollers.getInstance().setState(CoralRollersState.LEVEL1);
                }));

                // driverButtons.button(-1).onTrue((new SetWristState(WristStates.FOURTYFIVE,
                // ClosedLoopSlot.kSlot0)).withTimeout(1));//test withTimeout(can delete)
                // 13 TOTAL todos for setting actual value above
        }
}
