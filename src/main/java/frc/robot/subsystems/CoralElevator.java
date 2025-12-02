package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.common.LinearMechanism;
import frc.robot.subsystems.common.LinearMotorIO;
import frc.robot.subsystems.common.ElevatorMechanismIO;
import frc.robot.util.PIDConfig;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class CoralElevator extends LinearMechanism {
    // Constants
    private final MechanismGearing gearRatio = new MechanismGearing(2.0 * 12.0 / 1);
    private final Mass mass = Pounds.of(30);
    private final Distance outputRadius = Inches.of(1.751).div(2);

    private final Distance minPos = Meters.of(0.006);
    private final Distance maxPos = Meters.of(1.4605);
    private final Current maxCurrent = Amps.of(60.0);
    private final Time closedLoopRampRate = Seconds.of(.25);
    private final Time openLoopRampRate = Seconds.of(.25);

    private final LinearVelocity maxVel = MetersPerSecond.of(0.0508);
    private final LinearAcceleration maxAccel = MetersPerSecondPerSecond.of(0.02032);

    private final PIDConfig pid = new PIDConfig(0,0,0);
    private final ElevatorFeedforward ff = new ElevatorFeedforward(
        0.22643, 0.33045, 4.4162, 0.32248);

    /** Motor Configuration */
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withGearing(gearRatio)
            .withExternalEncoderGearing(gearRatio)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(pid.kP, pid.kI, pid.kD, maxVel,maxAccel)
            .withSimClosedLoopController(pid.kP, pid.kI, pid.kD, maxVel,maxAccel)
            .withFeedforward(ff)
            .withSimFeedforward(ff)
            .withTelemetry(getName() + " Motor", TelemetryVerbosity.HIGH)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(maxCurrent)
            .withClosedLoopRampRate(closedLoopRampRate)
            .withOpenLoopRampRate(openLoopRampRate);

    /** Motor Controller Initialization */
    private final SparkFlex sparkFlex = new SparkFlex(
            Constants.coralElevMotorID,
            MotorType.kBrushless);

    /** Smart Motor Controller Initialization */
    private final SmartMotorController motor = new SparkWrapper(
            sparkFlex, DCMotor.getNeoVortex(1), motorConfig);

    /** Elevator Mechanism Configuration */
    private final ElevatorConfig elevatorConfig = new ElevatorConfig(motor)
            .withSoftLimits(minPos, maxPos)
            .withHardLimits(minPos, maxPos)
            .withDrumRadius(outputRadius)
            .withMass(mass)
            .withTelemetry(getName(), TelemetryVerbosity.HIGH);

    /** Elevator Mechanism Initialization */
    private final Elevator arm = new Elevator(elevatorConfig);

    /** Elevator mechanism IO */
    private final LinearMotorIO io;

    /**
     * Constructor
     */
    public CoralElevator() {
        if (Constants.currentMode != Mode.REPLAY) {
            io = new ElevatorMechanismIO(arm);
        } else {
            io = new LinearMotorIO();
        }

        // Add preset positions
        io.addPreset("Travel", Meters.of(75.0));
        io.addPreset("Intake", Meters.of(85.5));
        io.addPreset("L1", Meters.of(73));
        io.addPreset("L2", Meters.of(60));
        io.addPreset("L3", Meters.of(60));
        io.addPreset("L4", Meters.of(60));
        io.addPreset("Algae", Meters.of(45));
    }

    @Override
    public LinearMotorIO getIO() {
        return io;
    }
}
