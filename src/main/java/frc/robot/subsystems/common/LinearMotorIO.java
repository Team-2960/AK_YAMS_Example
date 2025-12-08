package frc.robot.subsystems.common;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import org.littletonrobotics.junction.AutoLog;

public class LinearMotorIO extends MotorIO {

    private final HashMap<String, Distance> presetPos = new HashMap<>();
    private final HashMap<String, LinearVelocity> presetVel = new HashMap<>();

    @AutoLog
    public static class LinearMotorInputs extends MotorIO.MotorInputs {
        public MutDistance position = Meters.mutable(0);
        public MutLinearVelocity velocity = MetersPerSecond.mutable(0);
    }

    /**
     * Adds a new preset position. If the name already exists, the existing preset
     * will be overwritten
     *
     * @param name preset name
     * @param pos  preset position
     */
    public final void addPreset(String name, Distance pos) {
        presetPos.put(name, pos);
    }

    /**
     * Adds a new preset velocity. If the name already exists, the existing preset
     * will be overwritten
     *
     * @param name preset name
     * @param vel  preset velocity
     */
    public final void addPreset(String name, LinearVelocity vel) {
        presetVel.put(name, vel);
    }

    /**
     * Updates the set of loggable inputs
     *
     * @param inputs motor inputs object
     */
    public void updateInputs(LinearMotorInputs inputs) {
    }

    /**
     * Run the motor at the specified velocity.
     *
     * @param velocityRadPerSec target velocity
     */
    public void set(LinearVelocity output) {
    }

    /**
     * Run the turn motor to the specified rotation.
     *
     * @param rotation Target position
     */
    public void set(Distance output) {
    }

    /**
     * Moves to a named preset position
     *
     * @param name name the preset
     * @throws RuntimeException Thrown if name does not exist in the preset position
     *                          list
     */
    public void gotoPresetPos(String name) {
        if (presetPos.containsKey(name)) {
            set(presetPos.get(name));
        } else {
            throw new RuntimeException("No preset position with name " + name + "exists.");
        }
    }

    /**
     * Moves to a named preset position
     *
     * @param name name the preset
     * @throws RuntimeException Thrown if name does not exist in the preset position
     *                          list
     */
    public void gotoPresetVel(String name) {
        if (presetVel.containsKey(name)) {
            set(presetVel.get(name));
        } else {
            throw new RuntimeException("No preset velocity with name " + name + "exists.");
        }
    }

    /** Updates the telemetry */
    public void updateTelemetry() {
    };

    /** Updates the simulation */
    public void updateSimulation() {
    };

    /**
     * Gets a SysID command
     *
     * @param maxVoltage maximum voltage for sysID routine
     * @param step       voltage step size
     * @param duration   maximum duration of the sysID routine
     * @return command sequence to run sysID on the mechanism
     */
    public Command getSysIDCmd(Voltage maxVoltage, Velocity<VoltageUnit> step, Time duration) {
        return Commands.none();
    };
}
