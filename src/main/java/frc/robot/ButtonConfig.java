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
import frc.robot.Commands.ButtonLogger;
import frc.robot.Commands.Climb.SetClimbState;
import frc.robot.Commands.Climb.ZeroClimb;
import frc.robot.Commands.complex.AlignSource;
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
        driverRight.button(1).onTrue(new ButtonLogger("ScoreCoral attempted - driverRight", true, 1)).whileTrue(new ScoreCoral().onlyIf(
                () -> {
                    return CoralRollers.getInstance().HasCoral();
                }));
        driverRight.button(2).onTrue(new ButtonLogger("AlignCollect attempted - driverRight", true, 2)).whileTrue(new AlignCollect().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));
        // Score and Intake assistance buttons for right stick
        driverLeft.button(1).onTrue(new ButtonLogger("ScoreCoral attempted - driverLeft", true, 1)).whileTrue(new ScoreCoral().onlyIf(
                () -> {
                    return CoralRollers.getInstance().HasCoral();
                }));
        driverLeft.button(2).onTrue(new ButtonLogger("AlignCollect attempted - driverLeft", true, 2)).whileTrue(new AlignCollect().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));

        // Zero gyro button
        driverButtons.button(6).onTrue(new ButtonLogger("ZeroGyro attempted", true, 6)).onTrue(new InstantCommand(() -> {
            Swerve.getInstance().zeroGyro();
        }));

        coDriverButtons.button(3).onTrue(new ButtonLogger("CoralRollersState -> AlgaeOutput attempted", false, 3)).onTrue(new SetAlgaeRollerState(CoralRollersState.AlGAEOUTPUT).onlyIf(
            () -> {
                return !CoralRollers.getInstance().HasCoral();
            }
        ).withTimeout(1.5));
        coDriverButtons.button(2).onTrue(new ButtonLogger("EjectCoral attempted", false, 2)).onTrue(new EjectCoral());
        coDriverButtons.button(4).onTrue(new ButtonLogger("PrepReefPlacement attempted", false, 4)).onTrue(new PrepReefPlacement());

        coDriverButtons.button(5).onTrue(new ButtonLogger("Intake attempted", false, 5)).onTrue(new Intake().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));
        // Return robot to a safe configuration
        coDriverButtons.button(8).onTrue(new ButtonLogger("RobotSafeState attempted", false, 8)).onTrue(new RobotSafeState());
        coDriverButtons.button(9).onTrue(new ButtonLogger("RobotStop attempted", false, 9)).onTrue(new RobotStop());

        coDriverButtons.button(7).onTrue(new ButtonLogger("CollectAlgaeStack attempted", false, 7)).onTrue(new CollectAlgaeStack());
        // Reef mode active (Switch 20)
        // Reef selection
        coDriverButtons.button(10).and(coDriverButtons.button(20).negate()).onTrue(new ButtonLogger("ReefLevel 4 selected", false, 10)).onTrue(new ReefLevelSelection(4));
        coDriverButtons.button(12).and(coDriverButtons.button(20).negate()).onTrue(new ButtonLogger("ReefLevel 3 selected", false, 12)).onTrue(new ReefLevelSelection(3));
        coDriverButtons.button(14).and(coDriverButtons.button(20).negate()).onTrue(new ButtonLogger("ReefLevel 2 selected", false, 14)).onTrue(new ReefLevelSelection(2));
        coDriverButtons.button(16).and(coDriverButtons.button(20).negate()).onTrue(new ButtonLogger("ReefLevel 1 selected", false, 16)).onTrue(new ReefLevelSelection(1));
        coDriverButtons.button(17).and(coDriverButtons.button(20).negate()).onTrue(new ButtonLogger("Elephantiasis (jiggle intake) attempted", false, 17)).whileTrue(new Elephantiasis().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));
        coDriverRight.button(1).onTrue(new ButtonLogger("SetOrientation 0 attempted - coDriverRight", false, 1)).onTrue(new SetOrientation(0));
        coDriverRight.button(2).onTrue(new ButtonLogger("SetOrientation 1 attempted - coDriverRight", false, 2)).onTrue(new SetOrientation(1));
        // Climb mode active (Switch 20)
        coDriverButtons.button(14).and(coDriverButtons.button(20)).onTrue(new ButtonLogger("SetClimbState Stored attempted", false, 14)).whileTrue(new SetClimbState(ClimbStates.Stored));
        coDriverButtons.button(16).and(coDriverButtons.button(20)).onTrue(new ButtonLogger("SetClimbState Deployed attempted", false, 16)).whileTrue(new SetClimbState(ClimbStates.Deployed));
        coDriverButtons.button(14).and(coDriverButtons.button(20)).onFalse(new ButtonLogger("SetClimbState Climbing attempted (button released)", false, 14)).onFalse(new SetClimbState(ClimbStates.Climbing));
        coDriverButtons.button(16).and(coDriverButtons.button(20)).onFalse(new ButtonLogger("SetClimbState Climbing attempted (button released)", false, 16)).onFalse(new SetClimbState(ClimbStates.Climbing));
        coDriverButtons.button(17).and(coDriverButtons.button(20)).onTrue(new ButtonLogger("ZeroClimb attempted", false, 17)).whileTrue(new ZeroClimb());

        // reef align
        driverButtons.button(1).onTrue(new ButtonLogger("SetDriveState AlignReef attempted", true, 1)).whileTrue(new InstantCommand(() -> Swerve.getInstance().setDriveState(DriveState.AlignReef)));
        driverButtons.button(1).onFalse(new ButtonLogger("SetDriveState Manual attempted (button released)", true, 1)).onFalse(new InstantCommand(() -> {
            Swerve.getInstance().setDriveState(DriveState.Manual);
        }));
        driverButtons.button(2).onTrue(new ButtonLogger("EjectCoral attempted", true, 2))
                .onTrue(new EjectCoral());
        driverButtons.button(5).onTrue(new ButtonLogger("PrepReefPlacement attempted", true, 4))
                .onTrue(new PrepReefPlacement());
        driverButtons.button(4).onTrue(new ButtonLogger("Intake attempted", true, 5)).onTrue(new Intake().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));

        driverButtons.button(8).onTrue(new ButtonLogger("RobotSafeState attempted", true, 8)).onTrue(new RobotSafeState());
        driverButtons.button(9).onTrue(new ButtonLogger("RobotStop attempted", true, 9)).onTrue(new RobotStop());

        driverButtons.button(3).onTrue(new ButtonLogger("RemoveAlgaeBottom attempted", true, 3)).onTrue(new RemoveAlgaeBottom().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }
        ));
        driverButtons.button(7).onTrue(new ButtonLogger("RemoveAlgaeTop attempted", true, 7)).onTrue(new RemoveAlgaeTop().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }
        ));
        //coDriverButtons.button(11).onTrue(new ClearAlgae());
        //coDriverButtons.button(11).onFalse(new AlgaeInterupted());

        driverButtons.button(17).onTrue(new ButtonLogger("Elephantiasis (jiggle intake) attempted", true, 17)).whileTrue(new Elephantiasis().onlyIf(
                () -> {
                    return !CoralRollers.getInstance().HasCoral();
                }));

        // Climb mode active (Switch 20)
        driverButtons.button(14).and(driverButtons.button(20)).onTrue(new ButtonLogger("SetClimbState Stored attempted", true, 14)).whileTrue(new SetClimbState(ClimbStates.Stored));
        driverButtons.button(16).and(driverButtons.button(20)).onTrue(new ButtonLogger("SetClimbState Deployed attempted", true, 16)).whileTrue(new SetClimbState(ClimbStates.Deployed));
        driverButtons.button(14).and(driverButtons.button(20)).onFalse(new ButtonLogger("SetClimbState Climbing attempted (button released)", true, 14)).onFalse(new SetClimbState(ClimbStates.Climbing));
        driverButtons.button(16).and(driverButtons.button(20)).onTrue(new ButtonLogger("SetClimbState Climbing attempted (button released)", true, 16)).onFalse(new SetClimbState(ClimbStates.Climbing));
        driverButtons.button(17).and(driverButtons.button(20)).onTrue(new ButtonLogger("ZeroClimb attempted", true, 17)).whileTrue(new ZeroClimb());

    }
}
