package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.common.LinearMechanism;
import frc.robot.subsystems.common.LinearMotorIO;
import frc.robot.subsystems.common.RollerMechanismIO;
import frc.robot.util.PIDConfig;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class TankDrive {

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

    /**
     * Defines a single side of the drivetrain
     */
    private class LeftSide extends LinearMechanism {
        /** Motor Controller Initialization */
        private final SparkFlex frontMotor = new SparkFlex(Constants.occraLFDriveID, MotorType.kBrushless);
        private final SparkFlex rearMotor = new SparkFlex(Constants.occraLRDriveID, MotorType.kBrushless);

        /** Left Motor Configuration */
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
                .withTelemetry("Left Motor", TelemetryVerbosity.HIGH)
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(maxCurrent)
                .withClosedLoopRampRate(closedLoopRampRate)
                .withOpenLoopRampRate(openLoopRampRate)
                .withFollowers(Pair.of(rearMotor, false));

        /** Smart Motor Controller Initialization */
        private final SmartMotorController motor = new SparkWrapper(
                frontMotor,
                DCMotor.getNeoVortex(1),
                motorConfig);

        /** Roller mechanism IO */
        private final LinearMotorIO io;

        public LeftSide() {
            if (Constants.currentMode != Mode.REPLAY) {
                io = new RollerMechanismIO(motor);
            } else {
                io = new LinearMotorIO();
            }
        }

        @Override
        public LinearMotorIO getIO() {
            return io;
        }
    }

        /**
     * Defines a single side of the drivetrain
     */
    private class RightSide extends LinearMechanism {
        /** Motor Controller Initialization */
        private final SparkFlex frontMotor = new SparkFlex(Constants.occraLFDriveID, MotorType.kBrushless);
        private final SparkFlex rearMotor = new SparkFlex(Constants.occraLRDriveID, MotorType.kBrushless);

        /** Left Motor Configuration */
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
                .withTelemetry("Left Motor", TelemetryVerbosity.HIGH)
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(maxCurrent)
                .withClosedLoopRampRate(closedLoopRampRate)
                .withOpenLoopRampRate(openLoopRampRate)
                .withFollowers(Pair.of(rearMotor, false));

        /** Smart Motor Controller Initialization */
        private final SmartMotorController motor = new SparkWrapper(
                frontMotor,
                DCMotor.getNeoVortex(1),
                motorConfig);

        /** Roller mechanism IO */
        private final LinearMotorIO io;

        public RightSide() {
            if (Constants.currentMode != Mode.REPLAY) {
                io = new RollerMechanismIO(motor);
            } else {
                io = new LinearMotorIO();
            }
        }

        @Override
        public LinearMotorIO getIO() {
            return io;
        }
    }


    private final LeftSide leftSide = new LeftSide();
    private final RightSide rightSide = new RightSide();



    /*****************************/
    /* Command Factories Methods */
    /*****************************/
    public Command joystickDriveCmd(DoubleSupplier left, DoubleSupplier right) {
        return Commands.parallel(
            leftSide.setVoltCmd(() -> maxDriveVoltage.times(left.getAsDouble())),
            rightSide.setVoltCmd(() -> maxDriveVoltage.times(right.getAsDouble())));
    }
}
