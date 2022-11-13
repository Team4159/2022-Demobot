package frc.robot.subsystems;
import java.util.Map;
import java.util.Map.Entry;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake extends SubsystemBase {
    private CANSparkMax armMotor;
    private CANSparkMax rollerSpark;
    private PIDController armPID;
    private ArmState armStateDesired;
    private RollerState rollerState;

    private DigitalInput limitSwitch;
    
    public Intake(){
        armMotor = new CANSparkMax(IntakeConstants.armSparkID, MotorType.kBrushless);
        rollerSpark = new CANSparkMax(IntakeConstants.rollerSparkID, MotorType.kBrushless);
        armMotor.setInverted(IntakeConstants.armInverted);
        rollerSpark.setInverted(IntakeConstants.rollerInverted);
        armPID = new PIDController(IntakeConstants.kp, IntakeConstants.ki, IntakeConstants.kd);
        armStateDesired = ArmState.OFF;
        rollerState = RollerState.OFF;
        limitSwitch = new DigitalInput(IntakeConstants.limitSwitchChannel);

        armMotor.getEncoder().setPosition(0);
    }
    @Override
    public void periodic(){
        if (limitSwitch.get()) {
            armMotor.getEncoder().setPosition(IntakeConstants.armLowSetpoint);
        }

        switch(armStateDesired) {
            case HIGH:
                setArmMotor(runArmPID(getArmSparkPosition(), IntakeConstants.armHighSetpoint));
                break;
            case LOW:
                setArmMotor(runArmPID(getArmSparkPosition(), IntakeConstants.armLowSetpoint));
                break;
            case LIMIT:
                setArmMotor(runArmPID(getArmSparkPosition(), IntakeConstants.armLimitSetpoint));
                break;
            case OFF:
                setArmMotor(0);
                break;
        }
        switch(rollerState) {
            case FORWARD:
                setRollerMotor(IntakeConstants.rollerForwardSpeed);
                break;
            case BACKWARD:
                setRollerMotor(IntakeConstants.rollerBackwardSpeed);
                break;
            case OFF:
                setRollerMotor(0);
                break;
        }
    }

    public ArmState getActualArmState() {
        double pos = getArmSparkPosition();
        Map<ArmState, Double> diffs = Map.of(
            ArmState.LOW, Math.abs(IntakeConstants.armLowSetpoint - pos),
            ArmState.LIMIT, Math.abs(IntakeConstants.armLimitSetpoint - pos),
            ArmState.HIGH, Math.abs(IntakeConstants.armHighSetpoint - pos)
        );

        return diffs.entrySet().stream() // Get the key-value pairs from the above map (Set<Entry<ArmState,Double>>)
          .filter((Entry<ArmState, Double> a) -> { // remove the elements that are too far from the tolerance
            return a.getValue() < IntakeConstants.tolerance;
        }).reduce((Entry<ArmState, Double> a, Entry<ArmState, Double> b) -> { // find the ArmState with the setpoint that's closest to the current position
            return b.getValue() < a.getValue() ? b : a;
        }).orElse(Map.entry(ArmState.OFF, 0d)) // if no ArmState is found, return 0
        .getKey(); // fetch the ArmState and disregard the difference
    }

    public void setDesiredArmState(ArmState newState) {
        armStateDesired = newState;
    }
    public void changeRollerState (RollerState newState){
        rollerState = newState;
    }
    public void setArmMotor(double speed){
        speed = MathUtil.clamp(speed, -0.5, 0.5);
        armMotor.setVoltage(speed*12);
    }
    public void setRollerMotor(double speed){
        //System.out.println(speed);
        rollerSpark.setVoltage(speed*12);
    }
    public double runArmPID(double currentPos, double setPoint){
        return armPID.calculate(currentPos, setPoint);
    }
    public double getArmSparkPosition() {
        System.out.println("Arm Spark: " + armMotor.getEncoder().getPosition());
        return armMotor.getEncoder().getPosition();
    }
    public static enum ArmState {
        HIGH,
        LOW,
        LIMIT, // for the start of the match -- all the way up
        OFF
    }
    public static enum RollerState {
        FORWARD,
        BACKWARD,
        OFF
    }
}