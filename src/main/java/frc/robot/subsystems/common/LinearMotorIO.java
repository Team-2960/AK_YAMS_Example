package frc.robot.subsystems.common;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.AutoLog;

public interface LinearMotorIO {
    @AutoLog
    public static class LinearMotorInputs {
        public boolean connected = false;
        public MutDistance position = Meters.mutable(0);
        public MutLinearVelocity velocity = MetersPerSecond.mutable(0);
        public MutVoltage appliedVoltage = Volts.mutable(0);
        public MutCurrent current = Amps.mutable(0);
    }

    /**
     * Updates the set of loggable inputs
     *
     * @param inputs motor inputs object
     */
    public default void updateInputs(LinearMotorInputs inputs) {}

    /**
     * Run the motor at a specified open loop value
     *
     * @param output Output voltage to set
     */
    public default void set(Voltage output) {}

    /**
     * Run the motor at the specified velocity.
     *
     * @param velocityRadPerSec target velocity
     */
    public default void set(LinearVelocity velocityRadPerSec) {}

    /**
     * Run the turn motor to the specified rotation.
     *
     * @param rotation Target position
     */
    public default void set(Distance rotation) {}

    /**
     * Updates the telemetry
     */
    public default void updateTelemetry() {};

    /**
     * Updates the simulation
     */
    public default void updateSimulation() {};

    /**
     * Gets a SysID command
     * 
     * @param maxVoltage maximum voltage for sysID routine
     * @param step       voltage step size
     * @param duration   maximum duration of the sysID routine
     * @return command sequence to run sysID on the mechanism
     */
    public default Command getSysIDCmd(Voltage maxVoltage, Velocity<VoltageUnit> step, Time duration) {
        return Commands.none();
    };
}
