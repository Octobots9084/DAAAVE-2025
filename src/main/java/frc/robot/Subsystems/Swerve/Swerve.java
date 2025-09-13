// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Elevator.SetElevatorStateTolerance;
import frc.robot.Commands.ReefSelection.manager;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.auto.AlignCollect;
import frc.robot.Commands.auto.testPlace;
import frc.robot.Commands.complex.CollectCoral;
import frc.robot.Commands.complex.PrepCollect;
import frc.robot.Commands.complex.BargeThrow;
import frc.robot.Commands.auto.RemoveAlgaeInAuto;
import frc.robot.Commands.auto.testAlignInAuto;
import frc.robot.Commands.auto.testSuperCycleInAuto;
import frc.robot.States.ReefTargetLevel;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class Swerve extends SubsystemBase {

    SwerveIO io;
    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
    private static Swerve INSTANCE = null;

    public ReefTargetOrientation alignmentOrientation;
    public ReefTargetSide reefTargetSide;
    public boolean autoselectToggle = false;

    public static enum DriveState {
        None,
        Manual,
        AlignReef,
        AlignProcessor,
        AlignSource,
        Reverse
    };

    private DriveState driveState = DriveState.None;

    public boolean isAlignedToSource;
    public boolean isAlignedToCoralRight;
    public boolean isAlignedToCoralLeft;
    public boolean isAlignedToProcessor;
    public boolean isAlignedCenterReef; // TODO check if the algae removal needs to be centered on the reef
    private ReefTargetSide targetSide = ReefTargetSide.LEFT;
    private ReefTargetOrientation targetOrientation = ReefTargetOrientation.AB; // AB
    public static DriveState previousDriveState;
    public static boolean rotLock = true;

    public static Swerve getInstance() {
        if (INSTANCE == null) {
            throw new IllegalStateException("Swerve instance not set");
        }
        return INSTANCE;
    }

    public static Swerve setInstance(SwerveIO io) {
        INSTANCE = new Swerve(io);
        return INSTANCE;
    }

    public void setReefTargetSide(ReefTargetSide side) {

        if (manager.level == ElevatorStates.LEVEL1) {
            targetSide = ReefTargetSide.ALGAE;
            AlignVision.setPoleSide(ReefTargetSide.ALGAE);
        } else {
            targetSide = side;
            AlignVision.setPoleSide(side);
        }
    }

    public Optional<Pose2d> getSimPose() {
        return io.getSimPose();
    }

    public void setReefTargetOrientation(ReefTargetOrientation orientation) {
        targetOrientation = orientation;
        AlignVision.setReefOrientation(orientation);
    }

    public ReefTargetSide getReefTargetSide() {
        return targetSide;
    }

    public ReefTargetOrientation getReefTargetOrientation() {
        return targetOrientation;
    }

    public Swerve(SwerveIO io) {
        this.io = io;

    }

    public boolean getAutoselectState() {
        return autoselectToggle;
    }

    public void setAutoselectState(boolean theState) {
        autoselectToggle = theState;
    }

    public void configurePathplanner() {
        try {

            // NamedCommands.registerCommand("AlignSource", new
            // AlignSource().withTimeout(1));
            // NamedCommands.registerCommand("ScoreCoral_E_L4", new
            // ScoreCoral(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT,
            // ReefTargetOrientation.EF).withTimeout(0.75));
            // NamedCommands.registerCommand("ScoreCoral_D_L4", new
            // ScoreCoral(ElevatorStates.LEVEL4, ReefTargetSide.LEFT,
            // ReefTargetOrientation.CD).withTimeout(0.75));
            // NamedCommands.registerCommand("ScoreCoral_C_L4", new
            // ScoreCoral(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT,
            // ReefTargetOrientation.CD).withTimeout(0.75));
            // NamedCommands.registerCommand("ScoreCoral_B_L4", new
            // ScoreCoral(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT,
            // ReefTargetOrientation.AB).withTimeout(0.75));


            registerNamedCommands();
            
            RobotConfig config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder
            AutoBuilder.configure(
                    this::getPose,
                    this::resetPose,
                    this::getSpeeds,
                    (speeds, feedforwards) -> driveRobotRelativeAuto(speeds),
                    new PPHolonomicDriveController(
                            Constants.Swerve.translationConstants, Constants.Swerve.rotationConstants),
                    config,
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);

        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
    }

        public void registerNamedCommands () {
            registerSuperCycle();
            registerCoralPlace();
            registerMisc();
            registerTesting();
        }

            public void registerSuperCycle () {
                NamedCommands.registerCommand("SuperCycleB",
                        new testSuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.AB));
                NamedCommands.registerCommand("SuperCycleC",
                        new testSuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.CD));
                NamedCommands.registerCommand("SuperCycleD",
                        new testSuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.CD));
                NamedCommands.registerCommand("SuperCycleE",
                        new testSuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.EF));
                NamedCommands.registerCommand("SuperCycleF",
                        new testSuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.EF));
                NamedCommands.registerCommand("SuperCycleG",
                        new testSuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.GH));
                NamedCommands.registerCommand("SuperCycleH",
                        new testSuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.GH));
                NamedCommands.registerCommand("SuperCycleI",
                        new testSuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.IJ));
                NamedCommands.registerCommand("SuperCycleJ",
                        new testSuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.IJ));
                NamedCommands.registerCommand("SuperCycleK",
                        new testSuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.KL));
                NamedCommands.registerCommand("SuperCycleL",
                        new testSuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.KL));
                NamedCommands.registerCommand("SuperCycleA",
                        new testSuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.AB));
            }

            public void registerCoralPlace () {
                NamedCommands.registerCommand("placeB",
                    new testPlace(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.AB));
                NamedCommands.registerCommand("placeC",
                        new testPlace(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.CD));
                NamedCommands.registerCommand("placeG",
                        new testPlace(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.GH));
                NamedCommands.registerCommand("placeD",
                        new testPlace(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.CD));
                NamedCommands.registerCommand("placeEF",
                        new testPlace(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.EF));
                NamedCommands.registerCommand("placeI",
                        new testPlace(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.IJ));
                NamedCommands.registerCommand("placeJ",
                        new testPlace(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.IJ));
                NamedCommands.registerCommand("placeK",
                        new testPlace(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.KL));
                NamedCommands.registerCommand("placeL",
                        new testPlace(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.KL));
                NamedCommands.registerCommand("placeA",
                        new testPlace(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.AB));
            }

            public void registerMisc () {
                NamedCommands.registerCommand("InitalWristPos", new InstantCommand(() -> {
                    Wrist.getInstance().setState(WristStates.PREP, ClosedLoopSlot.kSlot0);
                }));
    
                NamedCommands.registerCommand("ScoreAlgae", new BargeThrow());
                NamedCommands.registerCommand("RemoveAlgaeInAutoL", new RemoveAlgaeInAuto(ReefTargetOrientation.KL));//HIGH
                NamedCommands.registerCommand("RemoveAlgaeInAutoC", new RemoveAlgaeInAuto(ReefTargetOrientation.CD));//LOW
                new EventTrigger("PrepWristPosition").onTrue(new InstantCommand(() -> {
                    Wrist.getInstance().setState(WristStates.PREP, ClosedLoopSlot.kSlot0);
                }));
                new EventTrigger("BringUpElevator").onTrue(new SetElevatorStateTolerance(ElevatorStates.LEVEL4, 5).andThen(new SetWristState(WristStates.L4, ClosedLoopSlot.kSlot0)));
                new EventTrigger("BringDownElevator").onTrue(new SetElevatorStateTolerance(ElevatorStates.LOW, 5).andThen(new SetWristState(WristStates.MANUAL, ClosedLoopSlot.kSlot0)));
                NamedCommands.registerCommand("AlignCollect", new AlignCollect());
                new EventTrigger("PrepCollect").onTrue(new PrepCollect());
            }

            public void registerTesting () {
                NamedCommands.registerCommand("AlignB",
                        new testAlignInAuto(ReefTargetSide.RIGHT, ReefTargetOrientation.AB, false));
                NamedCommands.registerCommand("AlignC",
                        new testAlignInAuto(ReefTargetSide.LEFT, ReefTargetOrientation.CD, false));
                NamedCommands.registerCommand("AlignD",
                        new testAlignInAuto(ReefTargetSide.RIGHT, ReefTargetOrientation.CD, false));
                NamedCommands.registerCommand("AlignE",
                        new testAlignInAuto(ReefTargetSide.LEFT, ReefTargetOrientation.EF, false));
                NamedCommands.registerCommand("AlignF",
                        new testAlignInAuto(ReefTargetSide.RIGHT, ReefTargetOrientation.EF, false));
                NamedCommands.registerCommand("AlignG",
                        new testAlignInAuto(ReefTargetSide.LEFT, ReefTargetOrientation.GH, false));
                NamedCommands.registerCommand("AlignH",
                        new testAlignInAuto(ReefTargetSide.RIGHT, ReefTargetOrientation.GH, false));
                NamedCommands.registerCommand("AlignI",
                        new testAlignInAuto(ReefTargetSide.LEFT, ReefTargetOrientation.IJ, false));
                NamedCommands.registerCommand("AlignJ",
                        new testAlignInAuto(ReefTargetSide.RIGHT, ReefTargetOrientation.IJ, false));
                NamedCommands.registerCommand("AlignK",
                        new testAlignInAuto(ReefTargetSide.LEFT, ReefTargetOrientation.KL, false));
                NamedCommands.registerCommand("AlignL",
                        new testAlignInAuto(ReefTargetSide.RIGHT, ReefTargetOrientation.KL, false));
                NamedCommands.registerCommand("AlignA",
                        new testAlignInAuto(ReefTargetSide.LEFT, ReefTargetOrientation.AB, false));

                NamedCommands.registerCommand("AlignAB",
                        new testAlignInAuto(ReefTargetSide.RIGHT, ReefTargetOrientation.GH, true));
                NamedCommands.registerCommand("AlignCD",
                        new testAlignInAuto(ReefTargetSide.LEFT, ReefTargetOrientation.IJ, true));
                NamedCommands.registerCommand("AlignEF",
                        new testAlignInAuto(ReefTargetSide.RIGHT, ReefTargetOrientation.IJ, true));
                NamedCommands.registerCommand("AlignGH",
                        new testAlignInAuto(ReefTargetSide.LEFT, ReefTargetOrientation.KL, true));
                NamedCommands.registerCommand("AlignIJ",
                        new testAlignInAuto(ReefTargetSide.RIGHT, ReefTargetOrientation.KL, true));
                NamedCommands.registerCommand("AlignKL",
                        new testAlignInAuto(ReefTargetSide.LEFT, ReefTargetOrientation.AB, true));
            }


    public SwerveIO getIo() {
        return io;
    }

    public void zeroGyro() {
        this.io.zeroGyro();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Swerve Drive", inputs);
        this.io.theHumbleAutoselectinator();
    }

    public double getGyro() {
        return this.io.getGyro();
    }

    public Pose2d getPose() {
        return this.io.getPose();
    }

    public void resetPose(Pose2d pose) {
        this.io.resetPose(pose);
    }

    public void setDriveState(DriveState state) {
        driveState = state;
    }

    public void setPreviousDriveState(DriveState state) {
        previousDriveState = state;
    }

    public DriveState getDriveState() {
        return driveState;
    }

    public DriveState getPreviousDriveState() {
        return previousDriveState;
    }

    public ChassisSpeeds getSpeeds() {
        return this.io.getSpeeds();
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        this.io.driveFieldRelative(fieldRelativeSpeeds);
    }

    public void driveRobotRelativeAuto(ChassisSpeeds fieldRelativeSpeeds) {
        this.io.driveRobotRelativeAuto(fieldRelativeSpeeds);
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        this.io.driveRobotRelative(robotRelativeSpeeds);
    }

    public SwerveModuleState[] getModuleStates() {
        return this.io.getModuleStates();
    }

    public SwerveModuleState[] getModuleDesiredStates() {
        return this.io.getModuleDesiredStates();
    }

    public SwerveModulePosition[] getPositions() {
        return this.io.getPositions();
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX
     *            Translation in the X direction.
     * @param translationY
     *            Translation in the Y direction.
     * @param headingX
     *            Heading X to calculate angle of the joystick.
     * @param headingY
     *            Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier headingX,
            DoubleSupplier headingY) {
        return run(
                () -> {
                    double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth control out
                    double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth control out
                    // Make the robot move
                    driveFieldRelative(
                            this.io.getTargetSpeeds(
                                    xInput, yInput, headingX.getAsDouble(), headingY.getAsDouble()));
                });
    }

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX
     *            Translation in the X direction.
     * @param translationY
     *            Translation in the Y direction.
     * @param angularRotationX
     *            Rotation of the robot to set
     * @return Drive command.
     */
    public Command driveCommand(
            DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(
                () -> {
                    // Make the robot move
                    this.io.driveRobotRelative(
                            new Translation2d(
                                    translationX.getAsDouble() * this.io.getMaximumChassisVelocity(),
                                    translationY.getAsDouble() * this.io.getMaximumChassisVelocity()),
                            angularRotationX.getAsDouble() * this.io.getMaximumChassisAngularVelocity(),
                            true,
                            false);
                });
    }

    public void addVisionReading(
            Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        this.io.addVisionReading(robotPose, timestamp, visionMeasurementStdDevs);
    }

    public void theHumbleAutoselectinator()
    {
        
    }
}