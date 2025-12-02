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
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
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

    public static final int coralElevMotorID = 11;

    public static final int algaeArmMotorID = 3;
    public static final int algaeRollerMotorID = 4;

    public static final int climberMotorID = 14;

    public static final int coralIntakePEID = 0;
}
