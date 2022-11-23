package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ArmIntake.ArmState;

public class Climber extends SubsystemBase {
    
    private CANSparkMax leftSpark, rightSpark;
    private TalonFX leftTalon, rightTalon;

    private PIDController leftSparkPID, rightSparkPID, leftTalonPID, rightTalonPID;

    private ArmState armState;
    private ElevatorState elevatorState;


    public Climber() {
        /* sparks */
        leftSpark = new CANSparkMax(
            ClimberConstants.leftSparkID,
            MotorType.kBrushless
        );
        leftSpark.setInverted(ClimberConstants.leftSparkInverted);
        leftSpark.setIdleMode(IdleMode.kBrake);

        rightSpark = new CANSparkMax(
            ClimberConstants.rightSparkID,
            MotorType.kBrushless
        );
        rightSpark.setInverted(ClimberConstants.rightSparkInverted);
        rightSpark.setIdleMode(IdleMode.kBrake);

        /* talons */
        leftTalon = new TalonFX(
            ClimberConstants.leftTalonID
        );
        leftTalon.setInverted(ClimberConstants.leftTalonInverted);
        leftTalon.setNeutralMode(NeutralMode.Brake);

        rightTalon = new TalonFX(
            ClimberConstants.rightTalonID
        );
        rightTalon.setInverted(ClimberConstants.rightTalonInverted);
        rightTalon.setNeutralMode(NeutralMode.Brake);

        /* PID Controllers */
        leftSparkPID = new PIDController(ClimberConstants.armkp, ClimberConstants.armki, ClimberConstants.armkd);
        rightSparkPID = new PIDController(ClimberConstants.armkp, ClimberConstants.armki, ClimberConstants.armkd);

        leftTalonPID = new PIDController(ClimberConstants.elevatorkp, ClimberConstants.elevatorki, ClimberConstants.elevatorkd);
        rightTalonPID = new PIDController(ClimberConstants.elevatorkp, ClimberConstants.elevatorki, ClimberConstants.elevatorkd);


        /* zero encoder values at robot init */
        leftSpark.getEncoder().setPosition(0);
        rightSpark.getEncoder().setPosition(0);
        
        leftTalon.setSelectedSensorPosition(0);
        rightTalon.setSelectedSensorPosition(0);

        armState = ArmState.OFF;
        elevatorState = ElevatorState.OFF;

    }

    @Override
    public void periodic() {
        
        switch (armState) {
            case OFF:
                runSparks(0, 0);
                break;
            case LOW:
                runSparks(
                    leftSparkPID.calculate(getSparkPosition(leftSpark), ClimberConstants.armLowSetpoint),
                    rightSparkPID.calculate(getSparkPosition(rightSpark), ClimberConstants.armLowSetpoint)
                );
                break;
            case HIGH:
                runSparks(
                    leftSparkPID.calculate(getSparkPosition(leftSpark), ClimberConstants.armHighSetpoint),
                    rightSparkPID.calculate(getSparkPosition(rightSpark), ClimberConstants.armHighSetpoint)
                );
                break;
            case MID:
                runSparks(
                    leftSparkPID.calculate(getSparkPosition(leftSpark), ClimberConstants.armMidSetpoint),
                    rightSparkPID.calculate(getSparkPosition(rightSpark), ClimberConstants.armMidSetpoint)
                );
            break;

            case DOWN:
                runSparks(
                    leftSparkPID.calculate(getSparkPosition(leftSpark), ClimberConstants.armDownSetpoint),
                    rightSparkPID.calculate(getSparkPosition(rightSpark), ClimberConstants.armDownSetpoint)
                );
                break;
        }


        System.out.println("L: " + getSparkPosition(leftSpark) + "R: " + getSparkPosition(rightSpark));



        switch (elevatorState) {
            case OFF:
                runTalons(0, 0);
                break;
            case LOW:
                runTalons(
                    leftTalonPID.calculate(getTalonPosition(leftTalon), ClimberConstants.elevatorLowSetpoint),
                    rightTalonPID.calculate(getTalonPosition(rightTalon), ClimberConstants.elevatorLowSetpoint)
                );
                break;
            case HIGH:
                runTalons(
                    leftTalonPID.calculate(getTalonPosition(leftTalon), ClimberConstants.elevatorHighSetpoint),
                    rightTalonPID.calculate(getTalonPosition(rightTalon), ClimberConstants.elevatorHighSetpoint)
                );
                break;
            case DOWN:
                runTalons(
                    leftTalonPID.calculate(getTalonPosition(leftTalon), ClimberConstants.elevatorDownSetpoint),
                    rightTalonPID.calculate(getTalonPosition(rightTalon), ClimberConstants.elevatorDownSetpoint)
                );
                break;
        }
    }

    public void runSparks(double leftSpeed, double rightSpeed) {
        leftSpark.setVoltage(leftSpeed*12);
        rightSpark.setVoltage(rightSpeed*12);
    }

    public double getSparkPosition(CANSparkMax spark) {
        return spark.getEncoder().getPosition();
    }

    
    public void runTalons(double leftSpeed, double rightSpeed) {
        leftTalon.set(TalonFXControlMode.PercentOutput, leftSpeed);
        rightTalon.set(TalonFXControlMode.PercentOutput, rightSpeed);
    }

    public double getTalonPosition(TalonFX talon) {
        return talon.getSelectedSensorPosition();
    }


    public void setArmState(ArmState newState) {
        armState = newState;
    }

    public void setElevatorState(ElevatorState newState) {
        elevatorState = newState;
    }

    public void toggleArm() {
        leftSparkPID.reset();
        rightSparkPID.reset();
        if (armState == ArmState.DOWN || armState ==  ArmState.OFF || armState == ArmState.LOW || armState == ArmState.HIGH) {
            armState = ArmState.MID;
        } else {
            armState = ArmState.LOW;
        }
    }


    // climber enums
    public static enum ElevatorState {
        DOWN,
        LOW,
        HIGH,
        OFF
    }

    public static enum ArmState {
        DOWN,
        LOW,
        HIGH,
        MID,
        OFF
    }

}
