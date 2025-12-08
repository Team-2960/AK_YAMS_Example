package frc.robot.subsystems.common;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public abstract class MotorMechanism extends SubsystemBase {

    protected abstract MotorIO getMotorIO();

    /*****************************/
    /* Command Factories Methods */
    /*****************************/

    /**
     * Gets a command to control the motor with a given voltage supplier
     *
     * @param volts voltage supplier
     * @return command to control the motor with a given voltage supplier
     */
    public Command setVoltCmd(Supplier<Voltage> volts) {
        return this.run(() -> getMotorIO().set(volts.get()));
    }

    /**
     * Gets a command to control the motor with a given voltage
     *
     * @param volts voltage
     * @return command to control the motor with a given voltage
     */
    public Command setVoltCmd(Voltage volts) {
        return this.run(() -> getMotorIO().set(volts));
    }

    /**
     * Gets a command to move to a preset voltage
     *
     * @param name name of a preset voltage
     * @return command to move to a preset voltage
     */
    public Command presetVoltCmd(String name) {
        return this.run(() -> getMotorIO().gotoPresetVolt(name));
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
        return getMotorIO().getSysIDCmd(maxVoltage, step, duration);
    }
}
