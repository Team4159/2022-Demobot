package frc.robot.subsystems;

import java.util.Map;
import java.util.Map.Entry;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

    private CANSparkMax lArm, rArm;
    private TalonFX lElev, rElev;

    private PIDController leftSparkPID, rightSparkPID, leftTalonPID, rightTalonPID;

    private ArmState armStateDesired;
    private ElevatorState elevatorState;

    public Climber() {
        /* sparks */
        lArm = new CANSparkMax(
                ClimberConstants.leftSparkID,
                MotorType.kBrushless);
        lArm.setInverted(ClimberConstants.leftSparkInverted);
        lArm.setIdleMode(IdleMode.kBrake);

        rArm = new CANSparkMax(
                ClimberConstants.rightSparkID,
                MotorType.kBrushless);
        rArm.setInverted(ClimberConstants.rightSparkInverted);
        rArm.setIdleMode(IdleMode.kBrake);

        /* talons */
        lElev = new TalonFX(
                ClimberConstants.leftTalonID);
        lElev.setInverted(ClimberConstants.leftTalonInverted);
        lElev.setNeutralMode(NeutralMode.Brake);

        rElev = new TalonFX(
                ClimberConstants.rightTalonID);
        rElev.setInverted(ClimberConstants.rightTalonInverted);
        rElev.setNeutralMode(NeutralMode.Brake);

        /* PID Controllers */
        leftSparkPID = new PIDController(ClimberConstants.armkp, ClimberConstants.armki, ClimberConstants.armkd);
        rightSparkPID = new PIDController(ClimberConstants.armkp, ClimberConstants.armki, ClimberConstants.armkd);

        leftTalonPID = new PIDController(ClimberConstants.elevatorkp, ClimberConstants.elevatorki,
                ClimberConstants.elevatorkd);
        rightTalonPID = new PIDController(ClimberConstants.elevatorkp, ClimberConstants.elevatorki,
                ClimberConstants.elevatorkd);

        /* zero encoder values at robot init */
        lArm.getEncoder().setPosition(0);
        rArm.getEncoder().setPosition(0);

        lElev.setSelectedSensorPosition(0);
        rElev.setSelectedSensorPosition(0);

        armStateDesired = ArmState.OFF;
        elevatorState = ElevatorState.OFF;
    }

    @Override
    public void periodic() {
        switch (armStateDesired) {
            case OFF:
                runSparks(0, 0);
                break;
            case LOW:
                runSparks(
                        leftSparkPID.calculate(getSparkPosition(lArm), ClimberConstants.armLowSetpoint),
                        rightSparkPID.calculate(getSparkPosition(rArm), ClimberConstants.armLowSetpoint));
                break;
            case HIGH:
                runSparks(
                        leftSparkPID.calculate(getSparkPosition(lArm), ClimberConstants.armHighSetpoint),
                        rightSparkPID.calculate(getSparkPosition(rArm), ClimberConstants.armHighSetpoint));
                break;
            case MID:
                runSparks(
                        leftSparkPID.calculate(getSparkPosition(lArm), ClimberConstants.armMidSetpoint),
                        rightSparkPID.calculate(getSparkPosition(rArm), ClimberConstants.armMidSetpoint));
                break;
            case DOWN:
                runSparks(
                        leftSparkPID.calculate(getSparkPosition(lArm), ClimberConstants.armDownSetpoint),
                        rightSparkPID.calculate(getSparkPosition(rArm), ClimberConstants.armDownSetpoint));
                break;
        }
        // System.out.println("L: " + getSparkPosition(leftSpark) + "R: " +
        // getSparkPosition(rightSpark));
        switch (elevatorState) {
            case OFF:
                runElev(0, 0);
                break;
            case LOW:
                runElev(
                        leftTalonPID.calculate(lElev.getSelectedSensorPosition(), ClimberConstants.elevatorLowSetpoint),
                        rightTalonPID.calculate(rElev.getSelectedSensorPosition(), ClimberConstants.elevatorLowSetpoint));
                break;
            case HIGH:
                runElev(
                        leftTalonPID.calculate(lElev.getSelectedSensorPosition(), ClimberConstants.elevatorHighSetpoint),
                        rightTalonPID.calculate(rElev.getSelectedSensorPosition(), ClimberConstants.elevatorHighSetpoint));
                break;
            case DOWN:
                runElev(
                        leftTalonPID.calculate(lElev.getSelectedSensorPosition(), ClimberConstants.elevatorDownSetpoint),
                        rightTalonPID.calculate(rElev.getSelectedSensorPosition(), ClimberConstants.elevatorDownSetpoint));
                break;
        }
    }

    public ArmState getActualArmState() {
        double pos = (getSparkPosition(lArm) + getSparkPosition(rArm)) / 2;

        Map<ArmState, Double> diffs = Map.of(
            ArmState.LOW, Math.abs(ClimberConstants.armLowSetpoint - pos),
            ArmState.MID, Math.abs(ClimberConstants.armMidSetpoint - pos),
            ArmState.HIGH, Math.abs(ClimberConstants.armHighSetpoint - pos)
        );

        return diffs.entrySet().stream() // Get the key-value pairs from the above map (Set<Entry<ArmState,Double>>)
          .filter((Entry<ArmState, Double> a) -> { // remove the elements that are too far from the tolerance
            return a.getValue() < ClimberConstants.tolerance;
        }).reduce((Entry<ArmState, Double> a, Entry<ArmState, Double> b) -> { // find the ArmState with the setpoint that's closest to the current position
            return b.getValue() < a.getValue() ? b : a;
        }).orElse(Map.entry(ArmState.OFF, 0d)) // if no ArmState is found, return 0
        .getKey(); // fetch the ArmState and disregard the difference
    }

    public void runSparks(double leftSpeed, double rightSpeed) {
        lArm.setVoltage(leftSpeed * 12);
        rArm.setVoltage(rightSpeed * 12);
    }

    public double getSparkPosition(CANSparkMax spark) {
        return spark.getEncoder().getPosition();
    }

    public void runElev(double leftSpeed, double rightSpeed) {
        lElev.set(TalonFXControlMode.PercentOutput, leftSpeed);
        rElev.set(TalonFXControlMode.PercentOutput, rightSpeed);
    }

    public void setArmState(ArmState newState) {
        leftSparkPID.reset();
        rightSparkPID.reset();
        armStateDesired = newState;
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
        MID,
        HIGH,
        OFF
    }
}
