package frc.robot.subsystems.common;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import yams.motorcontrollers.SmartMotorController;

public class RollerMechanismIO extends LinearMotorIO {
    /** Motor instance */
    private final SmartMotorController motor;

    /**
     * Constructor
     *
     * @param elevator elevator instance
     */
    public RollerMechanismIO(SmartMotorController motor) {
        this.motor = motor;
    }

    /** Updates the telemetry */
    @Override
    public void updateTelemetry() {
        motor.updateTelemetry();
    }

    /** Update Simulation */
    @Override
    public void updateSimulation() {
        motor.simIterate();
    }

    /**
     * Updates the set of loggable inputs
     *
     * @param inputs motor inputs object
     */
    @Override
    public void updateInputs(LinearMotorInputs inputs) {
        inputs.connected = true; // TODO Figure out how to check if a motor is connected using YAMS
        inputs.position.mut_replace(motor.getMeasurementPosition());
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
        motor.setVoltage(output);
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
        motor.setPosition(output);
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
        var sysId = motor.sysId(maxVoltage, step, duration);

        return Commands.sequence(
                sysId.quasistatic(Direction.kForward),
                sysId.quasistatic(Direction.kReverse),
                sysId.dynamic(Direction.kForward),
                sysId.dynamic(Direction.kReverse));
    }

    // TODO Allow distance limits for getSysIDCmd
}
