package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.util.MathUtil;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import swervelib.SwerveDrive;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveIOSystem implements SwerveIO {
    private SwerveDrive swerveDrive;
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    double maximumSpeed = 12;
    double maxTurnSpeed = 5;
    private Field2d field = new Field2d();

    public SwerveIOSystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory)
                    .createSwerveDrive(maximumSpeed, new Pose2d(3.1, 4, new Rotation2d(0)));
            swerveDrive.setHeadingCorrection(
                    false); // Heading correction should only be used while controlling the robot via
            // angle.
            swerveDrive.setCosineCompensator(
                    !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation
            // for simulations since it causes discrepancies not seen in real life.
            swerveDrive.setAngularVelocityCompensation(true, true, 0.07);
        } catch (IOException e) {
            e.printStackTrace();
        }

        SmartDashboard.putData("Field", field);
    }

    public void setMaxSpeed(double speed) {
        maximumSpeed = speed;
    }

    public double getMaxSpeed() {
        return maximumSpeed;
    }

    public void setMaxTurnSpeed(double speed) {
        maxTurnSpeed = speed;
    }

    public double getMaxTurnSpeed() {
        return maxTurnSpeed;
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public double getGyro() {
        return swerveDrive.getGyroRotation3d().getZ();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public Optional<Pose2d> getSimPose() {
        return swerveDrive.getSimulationDriveTrainPose();
    }

    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public ChassisSpeeds getSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        double maxAcceleration = getMaxAccelerationFromElevatorHeight();
        ChassisSpeeds limitedRobotRelativeSpeeds = MathUtil.limitXAndYAcceleration(robotRelativeSpeeds, getSpeeds(),
                maxAcceleration, maxAcceleration, 0.02);
        swerveDrive.drive(limitedRobotRelativeSpeeds);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        SmartDashboard.putString("FIELDRELATIVESPEDD", fieldRelativeSpeeds.toString());
        double maxAcceleration = getMaxAccelerationFromElevatorHeight();
        ChassisSpeeds limitedFieldRelativeSpeeds = MathUtil.limitXAndYAcceleration(fieldRelativeSpeeds,
                swerveDrive.getFieldVelocity(),
                maxAcceleration, maxAcceleration, 0.02);
        SmartDashboard.putString("limitedspeeds", fieldRelativeSpeeds.toString());

        swerveDrive.driveFieldOriented(limitedFieldRelativeSpeeds);
    }

    public double getMaxAccelerationFromElevatorHeight() {
        return (1.0 - (Elevator.getInstance().getPosition() / ElevatorStates.LEVEL4.position)) * 7.0 + 5.0;
    }

    public SwerveModuleState[] getModuleStates() {
        return swerveDrive.getStates();
    }

    public SwerveModulePosition[] getPositions() {
        return swerveDrive.getModulePositions();
    }

    public ChassisSpeeds getTargetSpeeds(
            double xInput, double yInput, double headingX, double headingY) {
        return swerveDrive.swerveController.getTargetSpeeds(
                xInput,
                yInput,
                headingX,
                headingY,
                swerveDrive.getYaw().getRadians(),
                swerveDrive.getMaximumChassisVelocity());
    }

    public double getMaximumChassisVelocity() {
        return swerveDrive.getMaximumChassisVelocity();
    }

    public double getMaximumChassisAngularVelocity() {
        return swerveDrive.getMaximumChassisAngularVelocity();
    }

    public void addVisionReading(
            Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        swerveDrive.addVisionMeasurement(new Pose2d(robotPose.getX(), robotPose.getY(), robotPose.getRotation()),
                timestamp, visionMeasurementStdDevs);
    }

    @Override
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    // public SwerveModuleState[] getModuleDesiredStates() {
    // return swerveDrive.getDesiredStates();
    // }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        SmartDashboard.putNumber("odometeryheading", swerveDrive.getOdometryHeading().getRadians());
        // TODO - Implement
        inputs.pose = this.getPose();
        inputs.speeds = this.getSpeeds();

        inputs.swerveModuleStates = this.getModuleStates();
        inputs.swerveModuleDesiredStates = this.getModuleDesiredStates();
        inputs.gyroAngleRadians = swerveDrive.getGyro().getRotation3d().toRotation2d().getRadians();
    }
}
