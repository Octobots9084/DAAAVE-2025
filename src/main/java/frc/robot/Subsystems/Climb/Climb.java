package frc.robot.Subsystems.Climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;

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
        // SmartDashboard.putBoolean("ClimbTopLimitSwitch", topLimitSwitch.get());
        // SmartDashboard.putBoolean("ClimbBottomLimitSwitch", bottomLimitSwitch.get());
    }

    public void setState(ClimbStates state, ClosedLoopSlot slot) {
        io.setPosition(state.position, slot);
    }

    public void setTalonVoltage(double voltage) {
        io.setTalonVoltage(voltage);
    }

    public ClimbStates getState() {
        return climbState;
    }

    public void updateSim() {
        io.updateSim();
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void allStop() {
        io.allStop();
    }

    public boolean talonIsStalled() {
        return io.talonIsStalled();
    }

    public boolean isAtState(ClimbStates state) {
        return MathUtil.isNear(io.getPosition(), state.position, 0.02);
    }

    public boolean isClimbClamped () {
        return io.isClimbClamped();
    }
}