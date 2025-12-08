package frc.robot.subsystems.common;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DifferentialDrive extends SubsystemBase {
    private final DifferentialDriveInputsAutoLogged inputs = new DifferentialDriveInputsAutoLogged();

    /**
     * Get the IO object for the mechanism
     *
     * @return the IO object for the mechanism
     */
    public abstract DifferentialDriveIO getIO();

    /** Update telemetry and inputs */
    @Override
    public void periodic() {
        var io = getIO();
        io.updateTelemetry();
        io.updateInputs(inputs);
    }

    /** Update the simulation */
    @Override
    public void simulationPeriodic() {
        getIO().updateSimulation();
    }

    /*****************************/
    /* Command Factories Methods */
    /*****************************/
    public Command joystickDriveCmd(DoubleSupplier left, DoubleSupplier right) {
        return this.run(
                () -> getIO().set(
                        left.getAsDouble(),
                        right.getAsDouble()));
    }

}
