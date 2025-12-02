package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.common.IntakeMechanismIO;
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

    private final AngularVelocity maxVel = DegreesPerSecond.of(90);
    private final AngularAcceleration maxAccel = DegreesPerSecondPerSecond.of(810);

    private final PIDConfig pid = new PIDConfig(.002,0,0);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(.02, .025, .029481);

    /** Motor Configuration */
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withGearing(gearRatio)
            .withExternalEncoderGearing(gearRatio)
            .withWheelDiameter(wheelDiam)
            .withMomentOfInertia(wheelDiam, mass)
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
            Constants.coralRollerMotorID,
            MotorType.kBrushless);

    /** Smart Motor Controller Initialization */
    private final SmartMotorController motor = new SparkWrapper(
            sparkFlex, DCMotor.getNeoVortex(1), motorConfig);

    /** Roller mechanism IO */
    private final LinearMotorIO io;

    /**
     * Constructor
     */
    public CoralRoller() {
        if (Constants.currentMode != Mode.REPLAY) {
            io = new IntakeMechanismIO(motor);
        } else {
            io = new LinearMotorIO();
        }
    }

    @Override
    public LinearMotorIO getIO() {
        return io;
    }
}
