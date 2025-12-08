package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.common.RollerMechanismIO;
import frc.robot.subsystems.common.LinearMechanism;
import frc.robot.subsystems.common.LinearMotorIO;
import frc.robot.util.PIDConfig;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class CoralRoller extends LinearMechanism {
    // Constants
    private final MechanismGearing gearRatio = new MechanismGearing(5.0 / 1);
    private final Mass mass = Pounds.of(1);
    private final Distance wheelDiam = Inches.of(2);

    private final Current maxCurrent = Amps.of(60.0);
    private final Time closedLoopRampRate = Seconds.of(.25);
    private final Time openLoopRampRate = Seconds.of(.25);

    private final LinearVelocity maxVel = InchesPerSecond.of(12);
    private final LinearAcceleration maxAccel = InchesPerSecond.per(Seconds).of(144);

    private final PIDConfig pid = new PIDConfig(.002, 0, 0);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(.02, .025, .029481);

    /** Motor Configuration */
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withGearing(gearRatio)
            .withExternalEncoderGearing(gearRatio)
            .withWheelDiameter(wheelDiam)
            .withMomentOfInertia(wheelDiam, mass)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(pid.kP, pid.kI, pid.kD, maxVel, maxAccel)
            .withSimClosedLoopController(pid.kP, pid.kI, pid.kD, maxVel, maxAccel)
            .withFeedforward(ff)
            .withSimFeedforward(ff)
            .withTelemetry(getName() + " Motor", TelemetryVerbosity.HIGH)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(maxCurrent)
            .withClosedLoopRampRate(closedLoopRampRate)
            .withOpenLoopRampRate(openLoopRampRate);

    /** Motor Controller Initialization */
    private final SparkFlex sparkFlex = new SparkFlex(Constants.coralRollerMotorID, MotorType.kBrushless);

    /** Smart Motor Controller Initialization */
    private final SmartMotorController motor = new SparkWrapper(sparkFlex, DCMotor.getNeoVortex(1), motorConfig);

    /** Roller mechanism IO */
    private final LinearMotorIO io;

    private final DigitalInput intakeSensor = new DigitalInput(Constants.coralIntakePEID);
    private final SparkLimitSwitch grippedSensor = sparkFlex.getForwardLimitSwitch();

    /** Constructor */
    public CoralRoller() {
        if (Constants.currentMode != Mode.REPLAY) {
            io = new RollerMechanismIO(motor);
        } else {
            io = new LinearMotorIO();
        }

        // Add Presets
        io.addPreset("Intake", Volts.of(-4.5));
        io.addPreset("Slow Intake", Volts.of(-1.5));
        io.addPreset("Eject", Volts.of(12));
        io.addPreset("Reverse", Volts.of(2));
        io.addPreset("Algae Removal", Volts.of(-5));
    }

    @Override
    public LinearMotorIO getIO() {
        return io;
    }

    /**
     * Creates trigger for the intake sensor
     *
     * @return trigger for the intake sensor
     */
    public Trigger getIntakeTrigger() {
        return new Trigger(this::coralAtIntake);
    }

    /**
     * Creates trigger for the gripper sensor
     *
     * @return trigger for the gripper sensor
     */
    public Trigger getGripperTrigger() {
        return new Trigger(this::coralInGripper);
    }

    /**
     * Checks if a coral is at the intake sensor
     *
     * @return true if a coral is at the intake, false otherwise
     */
    public boolean coralAtIntake() {
        return intakeSensor.get();
    }

    /**
     * Checks if a coral is in the gripper
     *
     * @return true if a coral is in the gripper, false otherwise
     */
    public boolean coralInGripper() {
        return grippedSensor.isPressed();
    }

    /*****************************/
    /* Command Factories Methods */
    /*****************************/
    /**
     * Get a command to automatically intake coral. Mechanism slows down when coral
     * is detected in
     * front of the gripped sensor.
     *
     * @return command to automatically intake coral
     */
    public Command autoIntakeCmd() {
        return Commands.sequence(
                Commands.deadline(
                        Commands.waitUntil(() -> !coralAtIntake() || coralInGripper()),
                        presetVoltCmd("Intake")),
                Commands.deadline(
                        Commands.waitUntil(() -> !coralAtIntake()), presetVoltCmd("Slow Intake")));
    }

    /**
     * Gets a command to automatically eject coral from the gripper.
     *
     * @return command to automatically eject coral from the gripper.
     */
    public Command autoEjectCmd() {
        return Commands.deadline(Commands.waitUntil(() -> !coralInGripper()), presetVoltCmd("Eject"));
    }
}
