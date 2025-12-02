package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.common.AngularMechanism;
import frc.robot.subsystems.common.AngularMotorIO;
import frc.robot.subsystems.common.ArmMechanismIO;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class CoralArm extends AngularMechanism {

    /** Motor Configuration */
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                    Constants.coralArmPID.kP,
                    Constants.coralArmPID.kI,
                    Constants.coralArmPID.kD,
                    Constants.coralArmMaxVel,
                    Constants.coralArmMaxAccel)
            .withSimClosedLoopController(
                    Constants.coralArmPID.kP,
                    Constants.coralArmPID.kI,
                    Constants.coralArmPID.kD,
                    Constants.coralArmMaxVel,
                    Constants.coralArmMaxAccel)
            .withFeedforward(Constants.coralArmFF)
            .withSimFeedforward(Constants.coralArmFF)
            .withTelemetry(getName() + " Motor", TelemetryVerbosity.HIGH)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Constants.coralArmMaxCurrent)
            .withClosedLoopRampRate(Constants.coralArmRampRate)
            .withOpenLoopRampRate(Constants.coralArmRampRate);

    /** Motor Controller Initialization */
    private final SparkFlex sparkFlex = new SparkFlex(
            Constants.coralArmMotorID,
            MotorType.kBrushless);

    /** Smart Motor Controller Initialization */
    private final SmartMotorController motor = new SparkWrapper(
            sparkFlex, DCMotor.getNeoVortex(1), motorConfig);

    /** Arm Mechanism Configuration */
    private final ArmConfig armConfig = new ArmConfig(motor)
            .withSoftLimits(
                    Constants.coralArmMinPos,
                    Constants.coralArmMaxPos)
            .withHardLimit(
                    Constants.coralArmMinPos,
                    Constants.coralArmMaxPos)
            .withStartingPosition(Constants.coralArmStartPos)
            .withLength(Constants.coralArmLength)
            .withMass(Constants.coralArmMass)
            .withTelemetry(getName(), TelemetryVerbosity.HIGH);

    /** Arm Mechanism Initialization */
    private final Arm arm = new Arm(armConfig);

    /** Arm mechanism IO */
    private final AngularMotorIO io;

    /**
     * Constructor
     */
    public CoralArm() {
        if(Constants.currentMode != Mode.REPLAY) {
            io = new ArmMechanismIO(arm);
        } else {
            io = new AngularMotorIO();
        }
        
        // Add preset positions
        io.addPreset("Travel", Degrees.of(75.0));
        io.addPreset("Intake", Degrees.of(85.5));
        io.addPreset("L1", Degrees.of(73));
        io.addPreset("L2", Degrees.of(60));
        io.addPreset("L3", Degrees.of(60));
        io.addPreset("L4", Degrees.of(60));
        io.addPreset("Algae", Degrees.of(45));
    }

    @Override
    public AngularMotorIO getIO() {
        return io;
    }
}
