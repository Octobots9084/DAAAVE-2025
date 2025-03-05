// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.CoralRollers.CoralRollersManual;
import frc.robot.Commands.Elevator.ElevatorManual;
import frc.robot.Commands.Wrist.WristManual;
import frc.robot.Commands.swerve.drivebase.TeleopDrive;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbIO;
import frc.robot.Subsystems.Climb.ClimbIOSystems;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersIO;
import frc.robot.Subsystems.CoralRollers.CoralRollersIOSim;
import frc.robot.Subsystems.CoralRollers.CoralRollersIOSystems;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOSim;
import frc.robot.Subsystems.Elevator.ElevatorIOSparkMax;
import frc.robot.Subsystems.Lights.Light;
import frc.robot.Subsystems.Lights.LightsIOSystem;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveIO;
import frc.robot.Subsystems.Swerve.SwerveIOSystem;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Vision.VisionIOSim;
import frc.robot.Subsystems.Vision.VisionIOSystem;
import frc.robot.Subsystems.Vision.VisionSubsystem;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristIO;
import frc.robot.Subsystems.Wrist.WristIOSim;
import frc.robot.Subsystems.Wrist.WristIOSparkMax;

import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    // The robot's subsystems and commands are defined here...
    private CoralRollers coralRollers;
    private Elevator elevator;
    private Wrist wrist;
    private Swerve swerve;
    private VisionSubsystem vision;
    private AlignVision alignVision;
    private Climb climb;
    private Light lights;

    private CoralRollersManual coralRollersManual;
    private ElevatorManual elevatorManual;
    private WristManual wristManual;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                Optional<Alliance> ally = DriverStation.getAlliance();
                if (ally.isPresent()) {
                    if (ally.get() == Alliance.Red) {
                        Constants.isBlueAlliance = false;
                    }
                    if (ally.get() == Alliance.Blue) {
                        Constants.isBlueAlliance = true;
                    }
                }

                CoralRollers.setInstance(new CoralRollersIOSystems());
                coralRollers = CoralRollers.getInstance();

                Elevator.setInstance(new ElevatorIOSparkMax());
                elevator = Elevator.getInstance();

                Wrist.setInstance(new WristIOSparkMax());
                wrist = Wrist.getInstance();

                Swerve.setInstance(new SwerveIOSystem());
                swerve = Swerve.getInstance();

                VisionSubsystem.setInstance(new VisionIOSystem());
                vision = VisionSubsystem.getInstance();

                alignVision = AlignVision.getInstance();

                Climb.setInstance(new ClimbIOSystems());
                climb = Climb.getInstance();

                Light.setInstance(new LightsIOSystem());
                lights = Light.getInstance();

                swerve.configurePathplanner();

                break;

            case SIM:
                Wrist.setInstance(new WristIOSim());
                wrist = Wrist.getInstance();
                Elevator.setInstance(new ElevatorIOSim());
                elevator = Elevator.getInstance();
                Swerve.setInstance(new SwerveIOSystem());
                swerve = Swerve.getInstance();
                
                CoralRollers.setInstance(
                        new CoralRollersIOSim(swerve.getIo().getSwerveDrive().getMapleSimDrive().get()));
                coralRollers = CoralRollers.getInstance();

                VisionSubsystem.setInstance(new VisionIOSim());
                vision = VisionSubsystem.getInstance();

                swerve.configurePathplanner();

                SimulatedArena.getInstance().resetFieldForAuto();
                break;

            case REPLAY:
                Climb.setInstance(new ClimbIO() {});
                climb = Climb.getInstance();
                CoralRollers.setInstance(new CoralRollersIO() {});
                coralRollers = CoralRollers.getInstance();

                Elevator.setInstance(new ElevatorIO() {});
                elevator = Elevator.getInstance();

                Wrist.setInstance(new WristIO() {});
                wrist = Wrist.getInstance();

                Swerve.setInstance(new SwerveIO() {});
                swerve = Swerve.getInstance();

                break;
            default:
                break;
        }

        int negative = Constants.isBlueAlliance ? -1 : 1;

        TeleopDrive closedFieldRel = new TeleopDrive(
                () -> Math.pow(MathUtil.applyDeadband(
                        negative * ButtonConfig.driverLeft.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND) / 2.,3.0),
                () -> Math.pow(MathUtil.applyDeadband(
                        negative * ButtonConfig.driverLeft.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND) / 2.,3),
                () -> MathUtil.applyDeadband(
                        -ButtonConfig.driverRight.getRawAxis(0), OperatorConstants.RIGHT_X_DEADBAND) / 3.);
        swerve.setDefaultCommand(closedFieldRel);

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
        // VisionSubsystem.getInstance();
        ButtonConfig buttons = new ButtonConfig();
        buttons.initTeleop();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
