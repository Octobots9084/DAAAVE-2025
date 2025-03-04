package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.States.ReefTargetLevel;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Commands.AlgaeRollers.SetAlgaeRollersState;
import frc.robot.Commands.Climb.SetClimbState;
import frc.robot.Commands.Climb.ZeroClimb;
import frc.robot.Commands.complex.AlignSource;
import frc.robot.Commands.complex.CancelAllCommands;
import frc.robot.Commands.complex.ClearAlgae;
import frc.robot.Commands.complex.EjectCoral;
import frc.robot.Commands.complex.Elephantiasis;
// import frc.robot.Commands.complex.ClearAlgae;
import frc.robot.Commands.complex.Intake;
import frc.robot.Commands.complex.PrepReefPlacement;
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
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.ReefSelection.ReefLevelSelection;
import frc.robot.Commands.ReefSelection.SetOrientation;
import frc.robot.Commands.Wrist.PrepCoral;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristStateTolerance;
import frc.robot.Commands.auto.AlignCollect;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollersStates;
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
            () -> { return CoralRollers.getInstance().HasCoral(); }));
        driverRight.button(2).onTrue(new AlignCollect().onlyIf(
            () -> { return !CoralRollers.getInstance().HasCoral(); }));
        // Score and Intake assistance buttons for right stick
        driverLeft.button(1).whileTrue(new ScoreCoral().onlyIf(
            () -> { return CoralRollers.getInstance().HasCoral(); }));
        driverLeft.button(2).onTrue(new AlignCollect().onlyIf(
            () -> { return !CoralRollers.getInstance().HasCoral(); }));

        // Zero gyro button
        driverButtons.button(6).onTrue(new InstantCommand(() -> {
            Swerve.getInstance().zeroGyro();
        }));

        coDriverButtons.button(2).onTrue(new EjectCoral());
        coDriverButtons.button(4).onTrue(new PrepReefPlacement());

        coDriverButtons.button(5).onTrue(new Intake().onlyIf(
            () -> { return !CoralRollers.getInstance().HasCoral(); }));
        // Return robot to a safe configuration
        coDriverButtons.button(8).onTrue(new RobotSafeState());
        coDriverButtons.button(9).onTrue(new RobotStop());

        // Reef mode active (Switch 20)
        // Reef selection
        coDriverButtons.button(10).and(coDriverButtons.button(20).negate()).onTrue(new ReefLevelSelection(4));
        coDriverButtons.button(12).and(coDriverButtons.button(20).negate()).onTrue(new ReefLevelSelection(3));
        coDriverButtons.button(14).and(coDriverButtons.button(20).negate()).onTrue(new ReefLevelSelection(2));
        coDriverButtons.button(16).and(coDriverButtons.button(20).negate()).onTrue(new ReefLevelSelection(1));
        coDriverButtons.button(17).and(coDriverButtons.button(20).negate()).whileTrue(new Elephantiasis().onlyIf(
            () -> {return !CoralRollers.getInstance().HasCoral();} ));
        coDriverRight.button(1).onTrue(new SetOrientation(0));
        coDriverRight.button(2).onTrue(new SetOrientation(1));
        // Climb mode active (Switch 20)
        coDriverButtons.button(14).and(coDriverButtons.button(20)).whileTrue(new SetClimbState(ClimbStates.Stored));
        coDriverButtons.button(16).and(coDriverButtons.button(20)).whileTrue(new SetClimbState(ClimbStates.Deployed));
        coDriverButtons.button(14).and(coDriverButtons.button(20)).onFalse(new SetClimbState(ClimbStates.Climbing));
        coDriverButtons.button(16).and(coDriverButtons.button(20)).onFalse(new SetClimbState(ClimbStates.Climbing));
        coDriverButtons.button(17).and(coDriverButtons.button(20)).whileTrue(new ZeroClimb());

        // reef align
        driverButtons.button(1).whileTrue(new InstantCommand(() -> Swerve.getInstance().setDriveState(DriveState.AlignReef)));
        driverButtons.button(1).onFalse(new InstantCommand(() -> {
            Swerve.getInstance().setDriveState(DriveState.Manual);
        }));
        driverButtons.button(2)
                .onTrue(new EjectCoral());
        driverButtons.button(4)
                .onTrue(new PrepReefPlacement());
        driverButtons.button(5).onTrue(new Intake().onlyIf(
        () -> { return !CoralRollers.getInstance().HasCoral(); }));
        
        driverButtons.button(8).onTrue(new RobotSafeState());
        driverButtons.button(9).onTrue(new RobotStop());

        driverButtons.button(7).onTrue(new ClearAlgae());
        //coDriverButtons.button(11).onTrue(new ClearAlgae());
        //coDriverButtons.button(11).onFalse(new AlgaeInterupted());

        driverButtons.button(17).whileTrue(new Elephantiasis().onlyIf(
            () -> {return !CoralRollers.getInstance().HasCoral();} ));

        // Climb mode active (Switch 20)
        driverButtons.button(14).and(driverButtons.button(20)).whileTrue(new SetClimbState(ClimbStates.Stored));
        driverButtons.button(16).and(driverButtons.button(20)).whileTrue(new SetClimbState(ClimbStates.Deployed));
        driverButtons.button(14).and(driverButtons.button(20)).onFalse(new SetClimbState(ClimbStates.Climbing));
        driverButtons.button(16).and(driverButtons.button(20)).onFalse(new SetClimbState(ClimbStates.Climbing));
        driverButtons.button(17).and(driverButtons.button(20)).whileTrue(new ZeroClimb());

    }
}
