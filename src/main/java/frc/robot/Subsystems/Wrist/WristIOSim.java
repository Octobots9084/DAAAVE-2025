package frc.robot.Subsystems.Wrist;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class WristIOSim implements WristIO {
  DCMotorSim motorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.025, 1), DCMotor.getNeo550(1));
  PIDController simController = new PIDController(0, 0, 0);
  double targetPosition;

  @Override
  public void updateInputs(WristIOInputs inputs) {

    inputs.wristPositionRotations = motorSim.getAngularPositionRotations();
    inputs.wristVelocityRPM = motorSim.getAngularVelocityRPM();
    inputs.wristAppliedVolts = motorSim.getInputVoltage();
    inputs.wristCurrentAmps = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    simController.setPID(kP, kI, kD);
  }

  @Override
  public void setPosition(double position, ClosedLoopSlot slot) {
    targetPosition = position;

  }

  @Override
  public void updateSim() {
    motorSim.setInputVoltage(
        simController.calculate(motorSim.getAngularPositionRotations(), targetPosition));
    motorSim.update(0.02);
  }

}
