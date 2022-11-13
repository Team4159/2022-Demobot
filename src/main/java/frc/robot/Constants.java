package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.75);
        public static final double wheelBase = Units.inchesToMeters(18.75);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (8.14 / 1.0); //8.14:1    ---- L1
        public static final double angleGearRatio = (150 / 7.1); //150:7.1    ---- Mk4i

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.3;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = true;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final double angleOffset = 109.42;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final double angleOffset = 325.28;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final double angleOffset = 344.88;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final double angleOffset = 124.36;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

    public static final class JoystickConstants {

        public static final class Driver {
            public static final int driverPort = 0;
            
            public static final int zeroGyro = 5;
            public static final int defenseMode = 2;
        }
        
        public static final class Secondary {
            public static final int secondaryPort = 1;
            
            public static final int intakeAndArm = 1;
            public static final int shoot = 2;
 
            public static final int lowerArmNoSpin = 7;
            public static final int forceRaiseArm = 6;

            public static final int rollerIn = 8;

            public static final int raiseClimber = 3;
            public static final int swingArm = 4;

            public static final int climbMode = 13;


        }
    }

    public static final class IntakeConstants {

        public static final int armSparkID = 13;
        public static final int rollerSparkID = 14;

        public static final int limitSwitchChannel = 0;

        public static final boolean armInverted = false;
        public static final boolean rollerInverted = false;

        public static final double tolerance = 1;
        public static final double kp = 0.45;
        public static final double ki = 0.002;
        public static final double kd = 0.008;
        public static final double armLimitSetpoint = 5.2;
        public static final double armLowSetpoint = 32;
        public static final double armHighSetpoint = 5.2;

        public static final double rollerForwardSpeed = 1;
        public static final double rollerBackwardSpeed = -1;

    }

    public static final class ClimberConstants {

        public static final int leftSparkID = 9;
        public static final int rightSparkID = 10;

        public static final boolean leftSparkInverted = true;
        public static final boolean rightSparkInverted = false;

        public static final double armkp = 0.05;
        public static final double armki = 0.001;
        public static final double armkd = 0.0001;

        public static final double armDownSetpoint = 0;
        public static final double armLowSetpoint = 14.5;
        public static final double armMidSetpoint = 19;
        public static final double armHighSetpoint = 25.5;
        public static final double tolerance = 1;

        public static final int leftTalonID = 12;
        public static final int rightTalonID = 11;

        public static final boolean leftTalonInverted = true;
        public static final boolean rightTalonInverted = false;
        
        public static final double elevatorkp = 0.00001;
        public static final double elevatorki = 0.000001;
        public static final double elevatorkd = 0;

        public static double elevatorDownSetpoint = 0;
        public static double elevatorLowSetpoint = 10000;
        public static double elevatorHighSetpoint = 280000;

    }

}