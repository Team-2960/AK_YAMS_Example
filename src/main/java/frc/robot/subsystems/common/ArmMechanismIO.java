package frc.robot.subsystems.common;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;

public class ArmMechanismIO extends AngularMotorIO {
    /** Arm instance */
    private final Arm arm;

    private final SmartMotorController motor;

    /**
     * Constructor
     *
     * @param arm arm instance
     */
    public ArmMechanismIO(Arm arm) {
        this.arm = arm;
        motor = arm.getMotorController();
    }

    /** Updates the telemetry */
    @Override
    public void updateTelemetry() {
        arm.updateTelemetry();
    }

    /** Update Simulation */
    @Override
    public void updateSimulation() {
        arm.simIterate();
    }

    /**
     * Updates the set of loggable inputs
     *
     * @param inputs motor inputs object
     */
    @Override
    public void updateInputs(AngularMotorInputs inputs) {
        inputs.connected = true; // TODO Figure out how to check if a motor is connected using YAMS
        inputs.position.mut_replace(arm.getAngle());
        inputs.velocity.mut_replace(motor.getMechanismVelocity());
        inputs.appliedVoltage.mut_replace(motor.getVoltage());
        inputs.current.mut_replace(motor.getStatorCurrent());
    }

    /**
     * Run the motor at a specified open loop value
     *
     * @param output Output voltage to set
     */
    public void set(Voltage output) {
        arm.setVoltage(output);
    }

    /**
     * Run the motor at the specified velocity.
     *
     * @param velocityRadPerSec target velocity
     */
    public void set(AngularVelocity output) {
        motor.setVelocity(output);
    }

    /**
     * Run the turn motor to the specified rotation.
     *
     * @param rotation Target position
     */
    public void set(Angle output) {
        arm.setAngle(output);
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
        return arm.sysId(maxVoltage, step, duration);
    }
}
