package frc.robot.Commands.complex;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;

public class AlignToTarget extends Command {
  private final AlignVision alignVision;
  private final Swerve swerve;
  private double speed;
  private double lidarSpeed;
  private double gyroSpeed;
  private PIDController pidController;
  private PIDController lidarPIDController;
  private PIDController cameraDepthPIDController;
  private PIDController gyroPIDController;
  private double aveLidarDist;
  private double diffLidarDist;
  private double[] refPosition;

  public AlignToTarget() {
    this.alignVision = AlignVision.getInstance();
    this.swerve = Swerve.getInstance();
    this.pidController = new PIDController(2, 0, 0);
    this.lidarPIDController = new PIDController(2, 0, 0);
    this.cameraDepthPIDController = new PIDController(1.25, 0, 0);
    this.gyroPIDController = new PIDController(4, 0, 0);
    this.gyroPIDController.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void execute() {
    aveLidarDist = (alignVision.getRightLidarDistance() + alignVision.getLeftLidarDistance()) / 2;
    diffLidarDist = alignVision.getRightLidarDistance() - alignVision.getLeftLidarDistance();
    refPosition = alignVision.getReferenceRobotPosition(alignVision.getCamera());

    try {
      if (!Double.isNaN(refPosition[0])) {

        speed = pidController.calculate(refPosition[1], 0.1524);
        lidarSpeed = alignVision.areBothLidarsValid()
            ? lidarPIDController.calculate(aveLidarDist, .12)
            : cameraDepthPIDController.calculate(refPosition[0], 0.381);
        gyroSpeed = alignVision.areBothLidarsValid()
            ? -gyroPIDController.calculate(Math.asin(diffLidarDist / .605), 0)
            : gyroPIDController.calculate(swerve.getGyro(), Math.toRadians(-60));
      } else {
        speed = 0;
        lidarSpeed = 0;
        gyroSpeed = 0;
      }
    } catch (Exception e) {
      speed = 0;
      lidarSpeed = 0;
      gyroSpeed = 0;
    }

    swerve.driveRobotRelative(new ChassisSpeeds(-lidarSpeed, -speed, gyroSpeed));
  }
}
