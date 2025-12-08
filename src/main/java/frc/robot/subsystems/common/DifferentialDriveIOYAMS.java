package frc.robot.subsystems.common;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.*;
import yams.motorcontrollers.SmartMotorController;

public class DifferentialDriveIOYAMS extends DifferentialDriveIO{

    public final SmartMotorController leftMotor;
    public final SmartMotorController rightMotor;

    public final Voltage maxCtrlVoltage;
    public final LinearVelocity maxCtrlVelocity;

    public DifferentialDriveIOYAMS(
            SmartMotorController leftMotor,
            SmartMotorController rightMotor,
            Voltage maxCtrlVoltage,
            LinearVelocity maxCtrlVelocity) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.maxCtrlVoltage = maxCtrlVoltage;
        this.maxCtrlVelocity = maxCtrlVelocity;
    }

    public DifferentialDriveIOYAMS(
            SmartMotorController leftMotor,
            SmartMotorController rightMotor,
            Voltage maxCtrlVoltage) {
        this(
                leftMotor,
                rightMotor,
                maxCtrlVoltage,
                MetersPerSecond.of(5));
    }

    public DifferentialDriveIOYAMS(
            SmartMotorController leftMotor,
            SmartMotorController rightMotor,
            LinearVelocity maxCtrlVelocity) {
        this(
                leftMotor,
                rightMotor,
                Volts.of(12),
                maxCtrlVelocity);
    }

    public DifferentialDriveIOYAMS(
            SmartMotorController leftMotor,
            SmartMotorController rightMotor) {
        this(
                leftMotor,
                rightMotor,
                Volts.of(12),
                MetersPerSecond.of(5));
    }

    /**
     * Updates the inputs object
     *
     * @param inputs inputs object
     */
    public void updateInputs(DifferentialDriveInputs inputs) {
        inputs.leftVolts.mut_replace(leftMotor.getVoltage());
        inputs.leftDistance.mut_replace(leftMotor.getMeasurementPosition());
        inputs.leftVelocity.mut_replace(leftMotor.getMeasurementVelocity());
        inputs.rightVolts.mut_replace(rightMotor.getVoltage());
        inputs.rightDistance.mut_replace(rightMotor.getMeasurementPosition());
        inputs.rightVelocity.mut_replace(rightMotor.getMeasurementVelocity());
    }

    /**
     * Sets the drive voltage based on a percent value
     *
     * @param left  left drive value
     * @param right right drive value
     */
    public void percentVoltage(double left, double right) {
        leftMotor.setVoltage(maxCtrlVoltage.times(left));
        rightMotor.setVoltage(maxCtrlVoltage.times(right));
    }

    /**
     * Sets the drive voltage
     *
     * @param left  left drive value
     * @param right right drive value
     */
    public void set(Voltage left, Voltage right) {
        leftMotor.setVoltage(left);
        rightMotor.setVoltage(right);
    }

    /**
     * Sets the drive velocity
     *
     * @param left  left drive value
     * @param right right drive value
     */
    public void set(LinearVelocity left, LinearVelocity right) {
        leftMotor.setVelocity(left);
        rightMotor.setVelocity(right);
    }

    /** Updates the telemetry */
    public void updateTelemetry() {
        leftMotor.updateTelemetry();
        rightMotor.updateTelemetry();
    };

    /** Updates the simulation */
    public void updateSimulation() {
    };
}
