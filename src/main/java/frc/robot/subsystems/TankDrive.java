package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.common.DifferentialDrive;
import frc.robot.subsystems.common.DifferentialDriveIO;
import frc.robot.subsystems.common.DifferentialDriveIOYAMS;
import frc.robot.util.PIDConfig;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class TankDrive extends DifferentialDrive {

    // Constants
    private final MechanismGearing gearRatio = new MechanismGearing(8.45 / 1);
    private final Mass mass = Pounds.of(1);
    private final Distance wheelDiam = Inches.of(6);

    private final Current maxCurrent = Amps.of(60.0);
    private final Time closedLoopRampRate = Seconds.of(.25);
    private final Time openLoopRampRate = Seconds.of(.25);

    private final LinearVelocity maxVel = MetersPerSecond.of(5.36);
    private final LinearAcceleration maxAccel = MetersPerSecondPerSecond.of(5);

    private final Voltage maxDriveVoltage = Volts.of(12);

    private final PIDConfig pid = new PIDConfig(.002, 0, 0);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(.02, .025, .029481);

    /** Motor Controller Initialization */
    private final SparkFlex lfMotor = new SparkFlex(Constants.occraLFDriveID, MotorType.kBrushless);
    private final SparkFlex lrMotor = new SparkFlex(Constants.occraLRDriveID, MotorType.kBrushless);
    private final SparkFlex rfMotor = new SparkFlex(Constants.occraLFDriveID, MotorType.kBrushless);
    private final SparkFlex rrMotor = new SparkFlex(Constants.occraLRDriveID, MotorType.kBrushless);

    /** Left Motor Configuration */
    private final SmartMotorControllerConfig leftMotorConfig = new SmartMotorControllerConfig(this)
            .withGearing(gearRatio)
            .withExternalEncoderGearing(gearRatio)
            .withWheelDiameter(wheelDiam)
            .withMomentOfInertia(wheelDiam, mass)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(pid.kP, pid.kI, pid.kD, maxVel, maxAccel)
            .withSimClosedLoopController(pid.kP, pid.kI, pid.kD, maxVel, maxAccel)
            .withFeedforward(ff)
            .withSimFeedforward(ff)
            .withTelemetry("Left Motor", TelemetryVerbosity.HIGH)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(maxCurrent)
            .withClosedLoopRampRate(closedLoopRampRate)
            .withOpenLoopRampRate(openLoopRampRate)
            .withFollowers(Pair.of(lrMotor, false));

    /** Left Motor Configuration */
    private final SmartMotorControllerConfig rightMotorConfig = new SmartMotorControllerConfig(this)
            .withGearing(gearRatio)
            .withExternalEncoderGearing(gearRatio)
            .withWheelDiameter(wheelDiam)
            .withMomentOfInertia(wheelDiam, mass)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(pid.kP, pid.kI, pid.kD, maxVel, maxAccel)
            .withSimClosedLoopController(pid.kP, pid.kI, pid.kD, maxVel, maxAccel)
            .withFeedforward(ff)
            .withSimFeedforward(ff)
            .withTelemetry("Left Motor", TelemetryVerbosity.HIGH)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(maxCurrent)
            .withClosedLoopRampRate(closedLoopRampRate)
            .withOpenLoopRampRate(openLoopRampRate)
            .withFollowers(Pair.of(rrMotor, false));

    /** Smart Motor Controller Initialization */
    private final SmartMotorController leftMotor = new SparkWrapper(
            lfMotor,
            DCMotor.getNeoVortex(1),
            leftMotorConfig);

    private final SmartMotorController rightMotor = new SparkWrapper(
            rfMotor,
            DCMotor.getNeoVortex(1),
            rightMotorConfig);

    /** Roller mechanism IO */
    private final DifferentialDriveIO io;

    public TankDrive() {
        if (Constants.currentMode != Mode.REPLAY) {
            io = new DifferentialDriveIOYAMS(leftMotor, rightMotor, maxDriveVoltage, maxVel);
        } else {
            io = new DifferentialDriveIO();
        }
    }

    @Override
    public DifferentialDriveIO getIO() {
        return io;
    }
}
