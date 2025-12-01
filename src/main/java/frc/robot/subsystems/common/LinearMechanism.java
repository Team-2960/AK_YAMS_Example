package frc.robot.subsystems.common;

import java.util.function.Supplier;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearMechanism extends SubsystemBase {
    private final LinearMotorIO io;
    private final LinearMotorInputsAutoLogged inputs = new LinearMotorInputsAutoLogged();

    /**
     * Constructor
     * 
     * @param io mechanism IO object
     */
    public LinearMechanism(LinearMotorIO io) {
        this.io = io;
    }

    /**
     * Update telemetry and inputs
     */
    @Override
    public void periodic() {
        io.updateTelemetry();
        io.updateInputs(inputs);
    }

    /**
     * Update the simulation
     */
    @Override
    public void simulationPeriodic() {
        io.updateSimulation();
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
        return this.run(() -> io.set(pos));
    }

    /**
     * Gets a command to control the motor with a given voltage supplier
     * 
     * @param volts voltage supplier
     * @return command to control the motor with a given voltage supplier
     */
    public Command setVoltCmd(Supplier<Voltage> volts) {
        return this.run(() -> io.set(volts.get()));
    }

    /**
     * Gets a command to control the motor with a given voltage
     * 
     * @param volts voltage
     * @return command to control the motor with a given voltage
     */
    public Command setVoltCmd(Voltage volts) {
        return this.run(() -> io.set(volts));
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
        return io.getSysIDCmd(maxVoltage, step, duration);
    }
}
