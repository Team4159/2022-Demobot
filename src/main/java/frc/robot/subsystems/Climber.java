package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
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
        /*leftSpark = new CANSparkMax(
            Constants.ClimberConstants.leftSparkID,
            MotorType.kBrushless
        );
        leftSpark.setInverted(Constants.ClimberConstants.leftSparkInverted);

        rightSpark = new CANSparkMax(
            Constants.ClimberConstants.rightSparkID,
            MotorType.kBrushless
        );
        rightSpark.setInverted(Constants.ClimberConstants.rightSparkInverted);*/

        /* talons */
        leftTalon = new TalonFX(
            ClimberConstants.leftTalonID
        );
        leftTalon.setInverted(ClimberConstants.leftTalonInverted);

        rightTalon = new TalonFX(
            ClimberConstants.rightTalonID
        );
        rightTalon.setInverted(ClimberConstants.rightTalonInverted);

        /* PID Controllers */
        /*leftSparkPID = new PIDController(Constants.ClimberConstants.armkp, Constants.ClimberConstants.armki, Constants.ClimberConstants.armkd);
        rightSparkPID = new PIDController(Constants.ClimberConstants.armkp, Constants.ClimberConstants.armki, Constants.ClimberConstants.armkd);*/

        leftTalonPID = new PIDController(ClimberConstants.elevatorkp, ClimberConstants.elevatorki, ClimberConstants.elevatorkd);
        rightTalonPID = new PIDController(ClimberConstants.elevatorkp, ClimberConstants.elevatorki, ClimberConstants.elevatorkd);


        /* zero encoder values at robot init */
        /*leftSpark.getEncoder().setPosition(0);
        rightSpark.getEncoder().setPosition(0);*/
        
        leftTalon.setSelectedSensorPosition(0);
        rightTalon.setSelectedSensorPosition(0);

        armState = ArmState.OFF;
        elevatorState = ElevatorState.OFF;

    }

    @Override
    public void periodic() {
        
        /*switch (armState) {
            case OFF:
                runSparks(0, 0);
                break;
            case LOW:
                runSparks(
                    leftSparkPID.calculate(getSparkPosition(leftSpark), Constants.ClimberConstants.armLowSetpoint),
                    rightSparkPID.calculate(getSparkPosition(rightSpark), Constants.ClimberConstants.armLowSetpoint)
                );
                break;
            case HIGH:
                runSparks(
                    leftSparkPID.calculate(getSparkPosition(leftSpark), Constants.ClimberConstants.armHighSetpoint),
                    rightSparkPID.calculate(getSparkPosition(rightSpark), Constants.ClimberConstants.armHighSetpoint)
                );
                break;
            case DOWN:
                runSparks(
                    leftSparkPID.calculate(getSparkPosition(leftSpark), Constants.ClimberConstants.armDownSetpoint),
                    rightSparkPID.calculate(getSparkPosition(rightSpark), Constants.ClimberConstants.armDownSetpoint)
                );
                break;
        }*/


        System.out.println("L: " + getTalonPosition(leftTalon) + "R: " + getTalonPosition(rightTalon));
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
        OFF
    }

}
