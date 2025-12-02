package frc.robot.subsystems.common;

import java.util.function.Supplier;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LinearMechanism extends SubsystemBase {
    private final LinearMotorInputsAutoLogged inputs = new LinearMotorInputsAutoLogged();

    /**
     * Get the IO object for the mechanism
     * 
     * @return the IO object for the mechanism
     */
    public abstract LinearMotorIO getIO();

    /**
     * Update telemetry and inputs
     */
    @Override
    public void periodic() {
        var io = getIO();
        io.updateTelemetry();
        io.updateInputs(inputs);
    }

    /**
     * Update the simulation
     */
    @Override
    public void simulationPeriodic() {
        getIO().updateSimulation();
    }

    /*****************************/
    /* Command Factories Methods */
    /*****************************/

    /**
     * Gets a command to move to a position
     * 
     * @param pos position to move to
     * @return command to move to a position
     */
    public Command setPosCmd(Distance pos) {
        return this.run(() -> getIO().set(pos));
    }

    /**
     * Gets a command to run the mechanism at a velocity from a supplier
     * 
     * @param vel velocity supplier
     * @return command to run the mechanism at a velocity from a supplier
     */
    public Command setVelCmd(Supplier<LinearVelocity> vel) {
        return this.run(() -> getIO().set(vel.get()));
    }

    /**
     * Gets a command to run the mechanism at a velocity
     * 
     * @param vel velocity supplier
     * @return command to run the mechanism at a velocity
     */
    public Command setVelCmd(LinearVelocity vel) {
        return this.run(() -> getIO().set(vel));
    }

    /**
     * Gets a command to control the motor with a given voltage supplier
     * 
     * @param volts voltage supplier
     * @return command to control the motor with a given voltage supplier
     */
    public Command setVoltCmd(Supplier<Voltage> volts) {
        return this.run(() -> getIO().set(volts.get()));
    }

    /**
     * Gets a command to control the motor with a given voltage
     * 
     * @param volts voltage
     * @return command to control the motor with a given voltage
     */
    public Command setVoltCmd(Voltage volts) {
        return this.run(() -> getIO().set(volts));
    }

    /**
     * Gets a command to move to a preset position
     * 
     * @param name name of a preset position
     * @return command to move to a preset position
     */
    public Command presetPosCmd(String name) {
        return this.run(() -> getIO().gotoPresetPos(name));
    }

    /**
     * Gets a command to move to a preset velocity
     * 
     * @param name name of a preset velocity
     * @return command to move to a preset velocity
     */
    public Command presetVelCmd(String name) {
        return this.run(() -> getIO().gotoPresetVel(name));
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
        return getIO().getSysIDCmd(maxVoltage, step, duration);
    }
}
