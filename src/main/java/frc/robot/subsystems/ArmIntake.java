package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class ArmIntake extends SubsystemBase {
    private CANSparkMax armSpark;
    private CANSparkMax rollerSpark;
    private PIDController armPID;
    private RelativeEncoder armEncoder;
    private ArmState armState;
    private RollerState rollerState;

    private DigitalInput limitSwitch;
    
    public ArmIntake(){
        armSpark = new CANSparkMax(IntakeConstants.armSparkID, MotorType.kBrushless);
        rollerSpark = new CANSparkMax(IntakeConstants.rollerSparkID, MotorType.kBrushless);
        armSpark.setInverted(IntakeConstants.armInverted);
        rollerSpark.setInverted(IntakeConstants.rollerInverted);
        armPID = new PIDController(IntakeConstants.kp, IntakeConstants.ki, IntakeConstants.kd);
        armEncoder = armSpark.getEncoder();
        armState = ArmState.OFF;
        rollerState = RollerState.OFF;
        limitSwitch = new DigitalInput(IntakeConstants.limitSwitchChannel);

        armEncoder.setPosition(0);
    }
    @Override
    public void periodic(){
        System.out.println("Arm Spark: " + armEncoder.getPosition());
        if (limitSwitch.get()) {
            armEncoder.setPosition(IntakeConstants.armLowSetpoint);
        }

        switch(armState) {
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


    public void changeArmState (ArmState newState){
        armState = newState;
    }
    public void changeRollerState (RollerState newState){
        rollerState = newState;
    }
    public void setArmMotor(double speed){
        //System.out.println(speed);
        speed = MathUtil.clamp(speed, -0.5, 0.5);
        armSpark.setVoltage(speed*12);
    }
    public void setRollerMotor(double speed){
        //System.out.println(speed);
        rollerSpark.setVoltage(speed*12);
    }
    public double runArmPID(double currentPos, double setPoint){
        return armPID.calculate(currentPos, setPoint);
    }
    public double getArmSparkPosition() {
        System.out.println("Arm Spark: " + armEncoder.getPosition());
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