package frc.robot.Subsystems.Climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    public final ClimbIO io;
    private static Climb instance = null;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    private ClimbStates climbState = ClimbStates.Stored;

    public static Climb getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Climb instance not set");
        }
        return instance;
    }

    public static Climb setInstance(ClimbIO io) {
        instance = new Climb(io);
        return instance;
    }

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ClimbInputs", inputs);
    }

    public void setState(ClimbStates state) {
        climbState = state;
        if (state == ClimbStates.Deployed)
            io.setVoltage(10);
        else if (state == ClimbStates.Climbing)
            io.setVoltage(-10);
        else
            io.setVoltage(0);
        // DELETE above code when using positions once again
        // io.setPosition(state.position);
    }

    public ClimbStates getState() {
        return climbState;
    }

    public void updateSim() {
        io.updateSim();
    }

    public void zeroEncoder() {
        io.zeroEncoder();
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }
}
