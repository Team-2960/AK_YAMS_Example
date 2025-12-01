package frc.robot.subsystems.common;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;

public class ElevatorMechanismIO implements LinearMotorIO {
    /** Elevator instance */
    private final Elevator elevator;
    private final SmartMotorController motor;

    /**
     * Constructor
     *
     * @param elevator elevator instance
     */
    public ElevatorMechanismIO(Elevator elevator) {
        this.elevator = elevator;
        motor = elevator.getMotorController();
    }

    /**
     * Updates the telemetry
     */
    @Override
    public void updateTelemetry() {
        elevator.updateTelemetry();
    }

    /**
     * Update Simulation
     */
    @Override
    public void updateSimulation() {
        elevator.simIterate();
    }

    /**
     * Updates the set of loggable inputs
     *
     * @param inputs motor inputs object
     */
    @Override
    public void updateInputs(LinearMotorInputs inputs) {
        inputs.connected = true; // TODO Figure out how to check if a motor is connected using YAMS
        inputs.position.mut_replace(elevator.getHeight());
        inputs.velocity.mut_replace(motor.getMeasurementVelocity());
        inputs.appliedVoltage.mut_replace(motor.getVoltage());
        inputs.current.mut_replace(motor.getStatorCurrent());
    }

    /**
     * Run the motor at a specified open loop value
     *
     * @param output Output voltage to set
     */
    @Override
    public void set(Voltage output) {
        elevator.setVoltage(output);
    }

    /**
     * Run the motor at the specified velocity.
     *
     * @param velocityRadPerSec target velocity
     */
    @Override
    public void set(LinearVelocity output) {
        motor.setVelocity(output);
    }

    /**
     * Run the turn motor to the specified rotation.
     *
     * @param rotation Target position
     */
    @Override
    public void set(Distance output) {
        elevator.setHeight(output);
    }

    /**
     * Gets a SysID command
     * 
     * @param maxVoltage maximum voltage for sysID routine
     * @param step       voltage step size
     * @param duration   maximum duration of the sysID routine
     * @return command sequence to run sysID on the mechanism
     */
    public Command getSysIDCmd(Voltage maxVoltage, Velocity<VoltageUnit> step, Time duration) {
        return elevator.sysId(maxVoltage, step, duration);
    }
}
