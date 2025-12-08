package frc.robot.subsystems.common;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

public class DifferentialDriveIO {

    /**
     * Inputs class for Differential drive
     */
    @AutoLog
    public static class DifferentialDriveInputs {
        public MutVoltage leftVolts = Volts.mutable(0);
        public MutVoltage rightVolts = Volts.mutable(0);
        public MutDistance leftDistance = Meters.mutable(0);
        public MutDistance rightDistance = Meters.mutable(0);
        public MutLinearVelocity leftVelocity = MetersPerSecond.mutable(0);
        public MutLinearVelocity rightVelocity = MetersPerSecond.mutable(0);
    }

    /**
     * Updates the inputs object
     *
     * @param inputs inputs object
     */
    public void updateInputs(DifferentialDriveInputs inputs) {
    }

    /**
     * Sets the drive voltage based on a percent value
     *
     * @param left  left drive value
     * @param right right drive value
     */
    public void set(double left, double right) {
    }

    /**
     * Sets the drive voltage
     *
     * @param left  left drive value
     * @param right right drive value
     */
    public void set(Voltage left, Voltage right) {
    }

    /**
     * Sets the drive velocity
     *
     * @param left  left drive value
     * @param right right drive value
     */
    public void set(LinearVelocity left, LinearVelocity right) {
    }

    /** Updates the telemetry */
    public void updateTelemetry() {
    };

    /** Updates the simulation */
    public void updateSimulation() {
    };
}
