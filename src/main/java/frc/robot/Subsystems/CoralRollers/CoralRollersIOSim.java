package frc.robot.Subsystems.CoralRollers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.States.ReefTargetLevel;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

// import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

public class CoralRollersIOSim implements CoralRollersIO {
  CoralRollersIOSystems sparkMaxes = new CoralRollersIOSystems();
  CoralRollersState state = CoralRollersState.STOPPED;
  boolean hasCoralInClaw = false;
  DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.025, 1), DCMotor.getNeo550(1));
  AnalogInputSim rearBeamSim = new AnalogInputSim(2);
  AnalogInputSim mouthBeamSim = new AnalogInputSim(3);

  private final IntakeSimulation intakeSimulation;
  SwerveDriveSimulation drivetrain;

  public CoralRollersIOSim(SwerveDriveSimulation driveTrain) {
    this.drivetrain = driveTrain;
    // Here, create the intake simulation with respect to the intake on your real robot
    this.intakeSimulation =
        IntakeSimulation
            .InTheFrameIntake( // Specify the type of game pieces that the intake can collect
                "Coral",
                // Specify the drivetrain to which this intake is attached
                driveTrain,
                // Specify width of the intake
                // TODO - fix width
                Meters.of(0.7),
                // The intake is mounted on the back side of the chassis
                IntakeSimulation.IntakeSide.BACK,
                // The intake can hold up to 1 note
                1);

    intakeSimulation.startIntake();
  }

  @Override
  public void updateInputs(CoralRollersIOInputs inputs) {
    inputs.velocityRPM = motorSim.getAngularVelocityRPM();
    inputs.appliedVolts = motorSim.getInputVoltage();
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.hasCoral = this.hasCoral();
  }

  @Override
  public void setVoltage(double volts) {
    motorSim.setInput(MathUtil.clamp(volts, -12.0, 12.0));

    if (volts == CoralRollersState.INTAKING.voltage) {
        this.state = CoralRollersState.INTAKING;
    } else if (volts == CoralRollersState.STOPPED.voltage) {
        this.state = CoralRollersState.STOPPED;
    } else if (volts == CoralRollersState.OUTPUT.voltage) {
        this.state = CoralRollersState.OUTPUT;
    }
  }

  @Override
  public void updateSim() {
    Pose3d[] coralInRobot = {};
    motorSim.update(0.02);

    // coral in chute and intaking, so coral moves from chute to claw
    if (this.coralInChute() && state == CoralRollersState.INTAKING) {
        intakeSimulation.obtainGamePieceFromIntake();
        this.hasCoralInClaw = true;
    }

    // coral in claw is released
    if (this.hasCoral() && state == CoralRollersState.OUTPUT) {
        this.hasCoralInClaw = false;
      // removes algae from the algae intake rollers
      double dropHeight = Elevator.getInstance().getReefTargetLevel().position;
      double wristAngle = Wrist.getInstance().getState().wristPosition;

      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new ReefscapeCoralOnFly(
                  // Obtain robot position from drive simulation
                  this.drivetrain.getSimulatedDriveTrainPose().getTranslation(),
                  // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                  new Translation2d(0.35, 0),
                  // Obtain robot speed from drive simulation
                  this.drivetrain.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                  // Obtain robot facing from drive simulation
                  this.drivetrain.getSimulatedDriveTrainPose().getRotation(),
                  // The height at which the coral is ejected
                  Meters.of(dropHeight),
                  // The initial speed of the coral
                  // TODO - use actual speed
                  MetersPerSecond.of(2),
                  // The angle of the wrist
                  Degrees.of(wristAngle)));
    }
  }

  @Override
  public boolean hasCoral() {
    return this.hasCoralInClaw;
  }

  @Override
  public boolean coralInChute() {
    return this.intakeSimulation.getGamePiecesAmount() != 0;
  }
}
