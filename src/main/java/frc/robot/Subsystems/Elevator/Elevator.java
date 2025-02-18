package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States.ReefTargetLevel;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final double TOP_CROSSBAR_POS = 68.692;
    private final double BOT_CROSSBAR_POS = 47.666;
    private ElevatorStates targetLevel = ElevatorStates.LOW;
    public ElevatorStates driverDesiredElevatorStates;

    // TODO add actual input chanel
    public DigitalInput toplimitSwitch = new DigitalInput(0);
    public DigitalInput bottomlimitSwitch = new DigitalInput(1);

    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Elevator instance not set");
        }
        return instance;
    }

    public void setTargetState(ElevatorStates state) {
        targetLevel = state;
    }

    public double getPosition() {
        return this.io.getPosition();
    }

    public static Elevator setInstance(ElevatorIO io) {
        instance = new Elevator(io);
        return instance;
    }

    private Elevator(ElevatorIO io) {
        this.io = io;
        // this.io.configurePID(0.7, 0, 0);
    }

    public ElevatorIO getElevatorIo() {
        return io;
    }

    @Override
    public void periodic() {

        // double currentPosition = this.inputs.leftPositionRotations;

        // if (((targetLevel.position > this.BOT_CROSSBAR_POS && currentPosition <
        // this.BOT_CROSSBAR_POS) ||
        // (targetLevel.position < this.TOP_CROSSBAR_POS && currentPosition >
        // this.TOP_CROSSBAR_POS))
        // && Wrist.getInstance().IsInsideRobot()) {
        // io.setPosition(currentPosition, currentPosition);
        // }

        // if (bottomlimitSwitch.get()) {
        // // We are going up and top limit is tripped so stop
        // if (currentPosition < 0) {
        // io.setPosition(0, 0);
        // }
        // }
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setState(ElevatorStates state) {
        if (state.position < 0)
            state.position = 0;
        // io.setPosition(state.position);
        targetLevel = state;
        Logger.recordOutput("Elevator/State", state);
    }

    public ElevatorStates getTargetState() {

        return targetLevel;
    }

    public void manualSetTargetPosistion(double position) {
        if (position < 0)
            position = 0;
        io.setPosition(position);
    }

    public boolean isAtState(ElevatorStates state, double tolerance) {
        return MathUtil.isNear(this.inputs.leftPositionRotations, targetLevel.position, tolerance)
                || MathUtil.isNear(
                        this.inputs.rightPositionRotations, targetLevel.position, tolerance);
    }

    public boolean isAtState(double tolerance) {
        return MathUtil.isNear(this.inputs.leftPositionRotations, targetLevel.position, tolerance)
                || MathUtil.isNear(
                        this.inputs.rightPositionRotations, targetLevel.position, tolerance);
    }

    public void updateSim() {
        io.updateSim();
    }
}
