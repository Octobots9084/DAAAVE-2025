package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {

  DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(2), 0.025, 1), DCMotor.getNeo550(1));
  PIDController simController = new PIDController(0, 0, 0);
  double targetPosition = 0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftPositionRotations = motorSim.getAngularPositionRotations();
    inputs.leftVelocityRPM = motorSim.getAngularVelocityRPM();
    inputs.leftAppliedVolts = motorSim.getInputVoltage();
    inputs.leftCurrentAmps = motorSim.getCurrentDrawAmps();

    inputs.rightPositionRotations = motorSim.getAngularPositionRotations();
    inputs.rightVelocityRPM = motorSim.getAngularVelocityRPM();
    inputs.rightAppliedVolts = motorSim.getInputVoltage();
    inputs.rightCurrentAmps = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    simController.setPID(kP, kI, kD);
  }

  @Override
  public void setPosition(double leftPosition, double rightPosition) {
    targetPosition = leftPosition;
  }

  @Override
  public void updateSim() {
    motorSim.setInputVoltage(
        simController.calculate(motorSim.getAngularPositionRotations(), targetPosition));
    motorSim.update(0.02);
  }
}
