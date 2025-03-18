// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.swerve.drivebase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States.AlignState;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Vision.AlignSourceAuto;

import java.util.function.DoubleSupplier;
import frc.robot.util.MathUtil;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends Command {
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private ChassisSpeeds alignSourceAutoChassisSpeeds;
    private static Swerve swerveInstance = Swerve.getInstance();
    private static AlignVision alignInstance = AlignVision.getInstance();

    /**
     * Creates a new ExampleCommand.
     *
     * @param swerve
     *            The subsystem used by this command.
     */
    public TeleopDrive(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega) {
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.addRequirements(Swerve.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        int negative = Constants.isBlueAlliance ? -1 : 1;

        // TODO 𓃕
        switch (swerveInstance.getDriveState()) {
            case Manual:
                // double[] speeds = MathUtil.circleVectorFromSquare(vX.getAsDouble(),
                // vY.getAsDouble(), swerveInstance.getIo().getMaxSpeed());
                Swerve.getInstance()
                        .driveFieldRelative(
                                new ChassisSpeeds(
                                        negative * vX.getAsDouble() * swerveInstance.getIo().getMaxSpeed(),
                                        negative * vY.getAsDouble() * swerveInstance.getIo().getMaxSpeed(),
                                        omega.getAsDouble() * swerveInstance.getIo().getMaxTurnSpeed()));
                break;
            case Reverse:
                swerveInstance.driveRobotRelative(new ChassisSpeeds(-1.2, 0, omega.getAsDouble() * swerveInstance.getIo().getMaxTurnSpeed()));
                break;
            case AlignReef:
                swerveInstance.driveRobotRelative(alignInstance.getAlignChassisSpeeds(AlignState.Reef));
            case AlignProcessor:
                break;
            case AlignSource:
                // if (Constants.isInAuto && !AlignSourceAuto.getInstance().wrenchControlFromDriversForSourceAlign()) {
                //     // swerveInstance.driveFieldRelative(
                //     // new ChassisSpeeds(
                //     // vX.getAsDouble() * swerveInstance.getIo().getMaxSpeed(),
                //     // vY.getAsDouble() * swerveInstance.getIo().getMaxSpeed(),
                //     // AlignSourceAuto.getInstance().getAlignChassisSpeeds().omegaRadiansPerSecond));
                //     alignSourceAutoChassisSpeeds = AlignSourceAuto.getInstance().getAlignChassisSpeeds();
                //     swerveInstance.driveRobotRelative(
                //             new ChassisSpeeds(
                //                     alignSourceAutoChassisSpeeds.vxMetersPerSecond,
                //                     vY.getAsDouble() * swerveInstance.getIo().getMaxSpeed(),
                //                     alignSourceAutoChassisSpeeds.omegaRadiansPerSecond));
                // } else {
                swerveInstance.driveRobotRelative(alignInstance.getAlignChassisSpeeds(alignInstance.getAlignSourceSide()));
                // }
                break;
            default:
                Swerve.getInstance()
                        .driveFieldRelative(
                                new ChassisSpeeds(
                                        negative * vX.getAsDouble() * swerveInstance.getIo().getMaxSpeed(),
                                        negative * vY.getAsDouble() * swerveInstance.getIo().getMaxSpeed(),
                                        omega.getAsDouble() * swerveInstance.getIo().getMaxTurnSpeed()));
                break;
        }

        swerveInstance.setPreviousDriveState(swerveInstance.getDriveState());
    }
}
