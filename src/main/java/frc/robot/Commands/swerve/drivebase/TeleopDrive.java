// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.swerve.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends Command {
  private final DoubleSupplier vX;
  private final DoubleSupplier vY;
  private final DoubleSupplier omega;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public TeleopDrive(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega) {
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.addRequirements(Swerve.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Command", "command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("the code works!!", "nope sorry");
    Swerve.getInstance()
        .driveFieldRelative(
                new ChassisSpeeds(vX.getAsDouble(), vY.getAsDouble(), omega.getAsDouble()));
  }
}
