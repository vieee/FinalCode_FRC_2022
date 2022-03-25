// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalSource;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DrivingConstants {

        public static final double maxSpeed = 0.7;
        public static final double maxTurnSpeed = 0.5;
        public static final int FR_ID = 44;
        public static final int BR_ID = 33;
        public static final int FL_ID = 22;
        public static final int BL_ID = 11;
        public static final int neoCountsPerRevolution = 42;
        public static final double integralResetBound = 0.2;
        public static final double kPEncoder = 0.0001;
        public static final double kIEncoder = 0.00001;
        public static final double kDEncoder = 0.0001;
        public static final double kPNavX = 0.0001;
        public static final double kINavX = 0.00001;
        public static final double kDNavX = 0.0001;
        public static final double turnThreshold = 0.2;
        public static final double motionStopThreshold = 0.05;
        public static final double neoMotorWheelCircumference = 15.24;
        public static final double motorGearBoxRatio = 7.37;
        public static final int kAutoDriveForwardSpeed = 0;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 0.5;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = 0.5;

    }

    public static final class OIConstants {

        public static final int kDriverJoystickPort = 0;

        public static final int kArcadeDriveTurnAxis = 0;
        public static final int kArcadeDriveSpeedAxis = 1;
        // public static final int kClimberOnesAxis = 4;
        // public static final int kClimberTwosAxis = 5;

        public static final int intakerDOwn_A_ButtonNumber = 1;
        public static final int shootStopping_B_ButtonNumber = 2;
        public static final int feeder_X_ButtonNumber = 3;
        public static final int intaker_Y_ButtonNumber = 4;
        public static final int shootTesting_LB_ButtonNumber = 5;
        public static final int shoot_RB_ButtonNumber = 6;

        // public static final int intaker_A_ButtonNumber = 0;

    }

    public static final class ShooterConstants {

        public static final int shoot_ID = 55; // previously 1
        public static final int neoCountsPerRevolution = 42;
        public static final double deadband = 0.2;
        public static final double frontRollSpeed = 0.52;
        public static final double backRollSpeed = -0.52;
        public static final double frontFenderRollSpeed = 0.5;
        public static final double backFenderRollSpeed = -0.5;

        public static final double kP = 0.00012; // 0.00018000000272877514; // 0.0011
        public static final double kI = 3e-8; // 0.0; // 3e-8
        public static final double kD = 1.2; // 0.000009999999747378752; // 1.2
        public static final double kIz = 0.0;
        public static final double kFF = 0.00017; // 0.00016999999934341758
        public static final double kMaxOutput = 1.0;
        public static final double kMinOutput = -1.0;
        public static final double maxRPM = 6000.0;
        public static final double deadbandVelocity = 1500;
        public static double autonomousShooterRPM = 2000;

    }

    public static final class IntakeConstants {

        public static final int intake_ID = 60;
        public static final int poser_FID = 0;
        public static final int poser_BID = 0;
        public static final double deadband = 0.15;
        public static final double flowSpeed = -0.35;
        public static final double stopSpeed = 0;
        // public static final int feeder_ID = 29;
        // public static final double feederSpeed = -0.4;
        public static final double autonomousSpeed = 0;
        public static final double autonomousFeederSpeed = 0;

    }

    public static final class TestingShooterConstants {
        public static double testingShooterVelocity = 2000;
    }

    /*public static final class ClimberConstants {
        public static final int ones_ID = 6;
        public static final int twos_ID = 7;
        public static final int baseEncoder_startID = 6;
        public static final int baseEncoder_endID = 7;
        public static final double twos_movement_threshold = 0.15;
        public static final double twos_speed_modulator = 0.5;
        public static final double ones_movement_threshold = 0.1;
        public static final double ones_speed_modulator = 0.5;
        // public static final double ones_threshold_stoppage = 0.1
        public static final double ones_base_speed = 0.55;
        public static final double ones_threshold_stoppage = 0.1;
    }*/

    public static final DigitalSource left_encoder_port = null;
    public static final DigitalSource second_encoder_port = null;
    public static final int left_ID = 6;
    public static final int right_ID = 7;
    public static final double deadband_speed = 0.05;
    public static final double climber_speed_limiter = 0;
    public static final int left_motor_movement_metric = 0;
    public static final double left_motor_movement_kp = 0;
    public static final double deadband_motion = 0.05;
}
