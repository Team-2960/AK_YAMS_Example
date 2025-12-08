package frc.robot.subsystems.common;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.HashMap;

public abstract class MotorIO {

    private final HashMap<String, Voltage> presetVolt = new HashMap<>();

    public static class MotorInputs {
        public boolean connected = false;
        public MutVoltage appliedVoltage = Volts.mutable(0);
        public MutCurrent current = Amps.mutable(0);
    }

    /**
     * Adds a new preset voltage. If the name already exists, the existing preset
     * will be overwritten
     *
     * @param name preset name
     * @param volt preset voltage
     */
    public final void addPreset(String name, Voltage volt) {
        presetVolt.put(name, volt);
    }

    /**
     * Run the motor at a specified open loop value
     *
     * @param output Output voltage to set
     */
    public void set(Voltage output) {
    }

    /**
     * Moves to a named preset voltage
     *
     * @param name name the preset
     * @throws RuntimeException Thrown if name does not exist in the preset voltage
     *                          list
     */
    public void gotoPresetVolt(String name) {
        if (presetVolt.containsKey(name)) {
            set(presetVolt.get(name));
        } else {
            throw new RuntimeException("No preset voltage with name " + name + "exists.");
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
    public abstract Command getSysIDCmd(
            Voltage maxVoltage, Velocity<VoltageUnit> step, Time duration);;
}
