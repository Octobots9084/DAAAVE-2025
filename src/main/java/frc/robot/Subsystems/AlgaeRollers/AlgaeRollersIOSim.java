package frc.robot.Subsystems.AlgaeRollers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.littletonrobotics.junction.Logger;

public class AlgaeRollersIOSim implements AlgaeRollersIO {
    AnalogInputSim beamInputSim = new AnalogInputSim(0);
    SwerveDriveSimulation drivetrain;
    AlgaeRollersStates state = AlgaeRollersStates.OFF;
    DCMotorSim motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.025, 1), DCMotor.getNeo550(1));

    private final IntakeSimulation intakeSimulation;

    public AlgaeRollersIOSim(SwerveDriveSimulation driveTrain) {
        this.drivetrain = driveTrain;
        // Here, create the intake simulation with respect to the intake on your real robot
        this.intakeSimulation =
            IntakeSimulation
                .InTheFrameIntake( // Specify the type of game pieces that the intake can collect
                    "Algae",
                    // Specify the drivetrain to which this intake is attached
                    driveTrain,
                    // Specify width of the intake
                    // TODO - fix width
                    Meters.of(0.7),
                    // The intake is mounted on the back side of the chassis
                    IntakeSimulation.IntakeSide.LEFT,
                    // The intake can hold up to 1 note
                    1);
        this.intakeSimulation.startIntake();
    }

    @Override
    public void updateInputs(AlgaeRollersIOInputs inputs) {
        inputs.velocityRPM = motorSim.getAngularVelocityRPM();
        inputs.appliedVolts = motorSim.getInputVoltage();
        inputs.currentAmps = motorSim.getCurrentDrawAmps();
        inputs.beamValue = this.hasAlgae();
    }

    @Override
    public void setVoltage(double volts) {
        motorSim.setInput(MathUtil.clamp(volts, -12.0, 12.0));

        if (volts == AlgaeRollersStates.INTAKE.voltage) {
            intakeSimulation.startIntake();
            this.state = AlgaeRollersStates.INTAKE;
        } else if (volts == AlgaeRollersStates.OFF.voltage) {
            intakeSimulation.stopIntake();
            this.state = AlgaeRollersStates.OFF;
        } else if (volts == AlgaeRollersStates.OUTPUT.voltage) {
            intakeSimulation.stopIntake();
            this.state = AlgaeRollersStates.OUTPUT;
        }
    }

    @Override
    public void updateSim() {
        motorSim.update(0.02);
        Pose3d[] algaeInRobot = {};

        if (this.hasAlgae() && this.state == AlgaeRollersStates.OUTPUT) {
            // removes algae from the intake
            intakeSimulation.obtainGamePieceFromIntake();
            // removes algae from the algae intake rollers
            ReefscapeAlgaeOnFly.setHitNetCallBack(() -> System.out.println("ALGAE hits NET!"));
            // adds algae to the arena as having been output from the robot
            ReefscapeAlgaeOnFly.setHitNetCallBack(() -> System.out.println("ALGAE hits NET!"));
            SimulatedArena.getInstance()
                .addGamePieceProjectile(
                    new ReefscapeAlgaeOnFly(
                        this.drivetrain.getSimulatedDriveTrainPose().getTranslation(),
                        new Translation2d(-0.6, 0),
                        this.drivetrain.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        drivetrain
                            .getSimulatedDriveTrainPose()
                            .getRotation()
                            .plus(new Rotation2d(-Math.PI / 2)),
                        Meters.of(0.40), // initial height of the ball, in meters
                        MetersPerSecond.of(-0.5), // initial velocity, in m/s
                        Degrees.of(0)) // shooter angle
                    .withProjectileTrajectoryDisplayCallBack(
                        (poses) ->
                            Logger.recordOutput(
                                "successfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
                        (poses) ->
                            Logger.recordOutput(
                                "missedShotsTrajectory", poses.toArray(Pose3d[]::new))));
        }

        if (this.hasAlgae()) {
            algaeInRobot = new Pose3d[]{new Pose3d(drivetrain.getSimulatedDriveTrainPose()).plus(new Transform3d(0, 0.6, 0.4, new Rotation3d()))};
        }
        Logger.recordOutput("FieldSimulation/AlgaeInRobot", algaeInRobot);
    }

    @Override
    public boolean hasAlgae() {
        return intakeSimulation.getGamePiecesAmount() != 0;
    }
}
