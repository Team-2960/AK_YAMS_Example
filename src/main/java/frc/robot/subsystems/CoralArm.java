package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.common.AngularMechanism;
import frc.robot.subsystems.common.AngularMotorIO;
import frc.robot.subsystems.common.ArmMechanismIO;
import frc.robot.util.PIDConfig;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class CoralArm extends AngularMechanism {
  // Constants
  private final MechanismGearing gearRatio = new MechanismGearing(125.0 / 1);
  private final Mass mass = Pounds.of(10);
  private final Distance length = Inches.of(12);

  private final Angle minPos = Degrees.of(0);
  private final Angle maxPos = Degrees.of(90);
  private final Current maxCurrent = Amps.of(60.0);
  private final Time closedLoopRampRate = Seconds.of(.25);
  private final Time openLoopRampRate = Seconds.of(.25);

  private final AngularVelocity maxVel = DegreesPerSecond.of(90);
  private final AngularAcceleration maxAccel = DegreesPerSecondPerSecond.of(810);

  private final PIDConfig pid = new PIDConfig(.002, 0, 0);
  private final ArmFeedforward ff = new ArmFeedforward(.02, .025, .029481);

  /** Motor Configuration */
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withGearing(gearRatio)
          .withExternalEncoderGearing(gearRatio)
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
  private final SparkFlex sparkFlex =
      new SparkFlex(Constants.coralArmMotorID, MotorType.kBrushless);

  /** Smart Motor Controller Initialization */
  private final SmartMotorController motor =
      new SparkWrapper(sparkFlex, DCMotor.getNeoVortex(1), motorConfig);

  /** Arm Mechanism Configuration */
  private final ArmConfig armConfig =
      new ArmConfig(motor)
          .withSoftLimits(minPos, maxPos)
          .withHardLimit(minPos, maxPos)
          .withLength(length)
          .withMass(mass)
          .withTelemetry(getName(), TelemetryVerbosity.HIGH);

  /** Arm Mechanism Initialization */
  private final Arm arm = new Arm(armConfig);

  /** Arm mechanism IO */
  private final AngularMotorIO io;

  /** Constructor */
  public CoralArm() {
    if (Constants.currentMode != Mode.REPLAY) {
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
