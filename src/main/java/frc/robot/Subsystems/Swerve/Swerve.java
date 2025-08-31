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
import frc.robot.Commands.auto.SuperCycleInAuto;
import frc.robot.Commands.auto.PlaceCoralInAuto;
import frc.robot.Commands.complex.CollectCoral;
import frc.robot.Commands.complex.PrepCollect;
import frc.robot.Commands.complex.BargeThrow;
import frc.robot.Commands.auto.RemoveAlgaeInAuto;
import frc.robot.Commands.auto.SuperCycleInAuto;
import frc.robot.Commands.auto.testing.TestAlignAnyInAuto;
import frc.robot.Commands.auto.testing.TestAlignInAuto;
import frc.robot.States.ReefTargetLevel;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

import java.security.cert.TrustAnchor;
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

    public static enum DriveState {
        None,
        Manual,
        AlignReef,
        AlignProcessor,
        AlignSource,
        Reverse,
        AlignBarge
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

    public boolean ableToAlignBarge () {
        if (Constants.isBlueAlliance) {// <7.6, red = > 10
            return Swerve.getInstance().getPose().getTranslation().getMeasureX().magnitude() < 7.6;
        }
        return Swerve.getInstance().getPose().getTranslation().getMeasureX().magnitude() > 10;//red
        
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
            //REGISTER NAMED COMMANDS
            
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
            registerAlgaeRemoval();

            NamedCommands.registerCommand("TESTSUPERCYCLE",
                        new SuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.IJ));
            NamedCommands.registerCommand("TESTALIGNANY",
                        new TestAlignAnyInAuto(ReefTargetSide.RIGHT));
        }

            public void registerAlgaeRemoval () {
                NamedCommands.registerCommand("RemoveAlgaeAB", new RemoveAlgaeInAuto(ReefTargetOrientation.AB));
                NamedCommands.registerCommand("RemoveAlgaeCD", new RemoveAlgaeInAuto(ReefTargetOrientation.CD));
                NamedCommands.registerCommand("RemoveAlgaeEF", new RemoveAlgaeInAuto(ReefTargetOrientation.EF));
                NamedCommands.registerCommand("RemoveAlgaeGH", new RemoveAlgaeInAuto(ReefTargetOrientation.GH));
                NamedCommands.registerCommand("RemoveAlgaeIJ", new RemoveAlgaeInAuto(ReefTargetOrientation.IJ));
                NamedCommands.registerCommand("RemoveAlgaeKL", new RemoveAlgaeInAuto(ReefTargetOrientation.KL));
            }
        
            public void registerSuperCycle () {
                NamedCommands.registerCommand("SuperCycleB",
                        new SuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.AB));
                NamedCommands.registerCommand("SuperCycleC",
                        new SuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.CD));
                NamedCommands.registerCommand("SuperCycleD",
                        new SuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.CD));
                NamedCommands.registerCommand("SuperCycleE",
                        new SuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.EF));
                NamedCommands.registerCommand("SuperCycleF",
                        new SuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.EF));
                NamedCommands.registerCommand("SuperCycleG",
                        new SuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.GH));
                NamedCommands.registerCommand("SuperCycleH",
                        new SuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.GH));
                NamedCommands.registerCommand("SuperCycleI",
                        new SuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.IJ));
                NamedCommands.registerCommand("SuperCycleJ",
                        new SuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.IJ));
                NamedCommands.registerCommand("SuperCycleK",
                        new SuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.KL));
                NamedCommands.registerCommand("SuperCycleL",
                        new SuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.KL));
                NamedCommands.registerCommand("SuperCycleA",
                        new SuperCycleInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.AB));
            }

            public void registerCoralPlace () {
                NamedCommands.registerCommand("placeB",
                    new PlaceCoralInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.AB));
                NamedCommands.registerCommand("placeC",
                        new PlaceCoralInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.CD));
                NamedCommands.registerCommand("placeG",
                        new PlaceCoralInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.GH));
                NamedCommands.registerCommand("placeD",
                        new PlaceCoralInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.CD));
                NamedCommands.registerCommand("placeEF",
                        new PlaceCoralInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.EF));
                NamedCommands.registerCommand("placeI",
                        new PlaceCoralInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.IJ));
                NamedCommands.registerCommand("placeJ",
                        new PlaceCoralInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.IJ));
                NamedCommands.registerCommand("placeK",
                        new PlaceCoralInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.KL));
                NamedCommands.registerCommand("placeL",
                        new PlaceCoralInAuto(ElevatorStates.LEVEL4, ReefTargetSide.RIGHT, ReefTargetOrientation.KL));
                NamedCommands.registerCommand("placeA",
                        new PlaceCoralInAuto(ElevatorStates.LEVEL4, ReefTargetSide.LEFT, ReefTargetOrientation.AB));
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
                // new EventTrigger("BringDownElevator").onTrue(new SetElevatorStateTolerance(ElevatorStates.LOW, 5).andThen(new SetWristState(WristStates.MANUAL, ClosedLoopSlot.kSlot0)));
                NamedCommands.registerCommand("BringDownElevator", new SetElevatorStateTolerance(ElevatorStates.LOW, 5));
                NamedCommands.registerCommand("AlignCollect", new AlignCollect());
                new EventTrigger("PrepCollect").onTrue(new PrepCollect());
            }

            public void registerTesting () {
                NamedCommands.registerCommand("AlignB",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.RIGHT, ReefTargetOrientation.AB));
                NamedCommands.registerCommand("AlignC",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.LEFT, ReefTargetOrientation.CD));
                NamedCommands.registerCommand("AlignD",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.RIGHT, ReefTargetOrientation.CD));
                NamedCommands.registerCommand("AlignE",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.LEFT, ReefTargetOrientation.EF));
                NamedCommands.registerCommand("AlignF",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.RIGHT, ReefTargetOrientation.EF));
                NamedCommands.registerCommand("AlignG",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.LEFT, ReefTargetOrientation.GH));
                NamedCommands.registerCommand("AlignH",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.RIGHT, ReefTargetOrientation.GH));
                NamedCommands.registerCommand("AlignI",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.LEFT, ReefTargetOrientation.IJ));
                NamedCommands.registerCommand("AlignJ",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.RIGHT, ReefTargetOrientation.IJ));
                NamedCommands.registerCommand("AlignK",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.LEFT, ReefTargetOrientation.KL));
                NamedCommands.registerCommand("AlignL",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.RIGHT, ReefTargetOrientation.KL));
                NamedCommands.registerCommand("AlignA",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.LEFT, ReefTargetOrientation.AB));

                NamedCommands.registerCommand("AlignAB",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.ALGAE, ReefTargetOrientation.AB));
                NamedCommands.registerCommand("AlignCD",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.ALGAE, ReefTargetOrientation.CD));
                NamedCommands.registerCommand("AlignEF",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.ALGAE, ReefTargetOrientation.EF));
                NamedCommands.registerCommand("AlignGH",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.ALGAE, ReefTargetOrientation.GH));
                NamedCommands.registerCommand("AlignIJ",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.ALGAE, ReefTargetOrientation.IJ));
                NamedCommands.registerCommand("AlignKL",
                        new TestAlignInAuto(ElevatorStates.LEVEL1, ReefTargetSide.ALGAE, ReefTargetOrientation.KL));
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
}