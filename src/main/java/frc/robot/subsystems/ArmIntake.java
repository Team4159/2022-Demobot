package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmIntakeConstants;
import edu.wpi.first.math.controller.PIDController;

public class ArmIntake extends SubsystemBase {
    private CANSparkMax armSpark;
    private CANSparkMax rollerSpark;
    private PIDController armPID;
    private RelativeEncoder armEncoder;
    private ArmState armState;
    private RollerState rollerState;
    
    public ArmIntake(){
        armSpark = new CANSparkMax(ArmIntakeConstants.armSparkID, MotorType.kBrushless);
        rollerSpark = new CANSparkMax(ArmIntakeConstants.rollerSparkID, MotorType.kBrushless);
        armSpark.setInverted(ArmIntakeConstants.armInverted);
        rollerSpark.setInverted(ArmIntakeConstants.rollerInverted);
        armPID = new PIDController(ArmIntakeConstants.kp, ArmIntakeConstants.ki, ArmIntakeConstants.kd);
        armEncoder = armSpark.getEncoder();
        armState = ArmState.OFF;
        rollerState = RollerState.OFF;

        armEncoder.setPosition(0);
    }
    @Override
    public void periodic(){
        System.out.println("Arm Spark Pos: " + getArmSparkPosition());
        System.out.println("Arm state: " + armState);
        switch(armState) {
            case HIGH:
                setArmMotor(runArmPID(getArmSparkPosition(), ArmIntakeConstants.armHighSetpoint));
            case LOW:
                setArmMotor(runArmPID(getArmSparkPosition(), ArmIntakeConstants.armLowSetpoint));
            case LIMIT:
                setArmMotor(runArmPID(getArmSparkPosition(), ArmIntakeConstants.armLimitSetpoint));
            case OFF:
                setArmMotor(0);
        }
        switch(rollerState) {
            case FORWARD:
                setRollerMotor(ArmIntakeConstants.rollerForwardSpeed);
            case BACKWARD:
                setRollerMotor(ArmIntakeConstants.rollerBackwardSpeed);
            case OFF:
                setRollerMotor(0);
        }
    }


    public void changeArmState (ArmState newState){
        armState = newState;
    }
    public void changeRollerState (RollerState newState){
        rollerState = newState;
    }
    public void setArmMotor(double speed){
        armSpark.setVoltage(speed*12);
    }
    public void setRollerMotor(double speed){
        rollerSpark.setVoltage(speed*12);
    }
    public double runArmPID(double currentPos, double setPoint){
        return armPID.calculate(currentPos, setPoint);
    }
    public double getArmSparkPosition() {
        return armEncoder.getPosition();
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