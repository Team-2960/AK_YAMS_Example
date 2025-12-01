// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;


import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.PIDConfig;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }


    // CAN ID
    public static final int lfDriveMotorID = 9;
    public static final int lfAngleMotorID = 10;
    public static final int lfAngleEncoderID = lfAngleMotorID;

    public static final int rfDriveMotorID = 7;
    public static final int rfAngleMotorID = 8;
    public static final int rfAngleEncoderID = rfAngleMotorID;

    public static final int lrDriveMotorID = 1;
    public static final int lrAngleMotorID = 2;
    public static final int lrAngleEncoderID = lrAngleMotorID;

    public static final int rrDriveMotorID = 5;
    public static final int rrAngleMotorID = 6;
    public static final int rrAngleEncoderID = rrAngleMotorID;

    public static final int coralArmMotorID = 12;
    public static final int coralRollerMotorID = 13;

    public static final int elevatorMotorID = 11;

    public static final int algaeArmMotorID = 3;
    public static final int algaeRollerMotorID = 4;

    public static final int climberMotorID = 14;

    public static final int coralIntakePEID = 0;

    // Coral Arm Config
    public static final AngularVelocity coralArmMaxVel = DegreesPerSecond.of(90);
    public static final AngularAcceleration coralArmMaxAccel = DegreesPerSecondPerSecond.of(810);
    public static final Current coralArmMaxCurrent = Amps.of(60.0);
    public static final Time coralArmRampRate = Milliseconds.of(250);

    public static final Angle coralArmMinPos = Degrees.of(0);
    public static final Angle coralArmMaxPos = Degrees.of(90);
    public static final Angle coralArmStartPos = Degrees.of(85.5);

    public static final Distance coralArmLength = Inches.of(12); // Distance to Arm Center of Mass from pivot
    public static final Mass coralArmMass = Pounds.of(10);

    public static final Voltage coralArmSysIDMaxVolt = Volts.of(7);
    public static final Velocity<VoltageUnit> coralArmSysIDVoltStep =  Volts.of(2).per(Seconds);
    public static final Time coralArmSysIDDuration = Seconds.of(4);

    public static final PIDConfig coralArmPID = new PIDConfig(0.002, 0, 0);
    public static final ArmFeedforward coralArmFF = new ArmFeedforward(.02, .025, .029481);
}
