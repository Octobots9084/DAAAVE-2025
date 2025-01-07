package frc.robot.Subsystems.AlgaeRollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class AlgaeRollers extends SubsystemBase {
    private final AlgaeRollersIOSparkMax io;
    private final AlgaeRollersIOInputsAutoLogged inputs = new AlgaeRollersIOInputsAutoLogged();

    private static AlgaeRollers instance;

    public static void setInstance(AlgaeRollers subsystem) {
        AlgaeRollers.instance = subsystem;
    }

    public static AlgaeRollers getInstance() {
        return instance;
    }
    /** Creates a new Flywheel. */
    public AlgaeRollers(AlgaeRollersIOSparkMax io) {
        this.io = io;
        io.configurePID(8, 0, 0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeRollers", inputs);
    }

    /** Run closed loop at the specified velocity. */
    public void setState(AlgaeRollersStates state) {
        io.setVoltage(state.voltage);

        Logger.recordOutput("AlgaeRollers/State", state);
    }

    public boolean isAtState(AlgaeRollersStates state, double tolerance) {
        return MathUtil.isNear(state.voltage, io.getVoltage(), tolerance);
    }
}
