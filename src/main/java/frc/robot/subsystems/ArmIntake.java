package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmIntakeConstants;
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
        armSpark = new CANSparkMax(ArmIntakeConstants.armSparkID, MotorType.kBrushless);
        rollerSpark = new CANSparkMax(ArmIntakeConstants.rollerSparkID, MotorType.kBrushless);
        armSpark.setInverted(ArmIntakeConstants.armInverted);
        rollerSpark.setInverted(ArmIntakeConstants.rollerInverted);
        armPID = new PIDController(ArmIntakeConstants.kp, ArmIntakeConstants.ki, ArmIntakeConstants.kd);
        armEncoder = armSpark.getEncoder();
        armState = ArmState.OFF;
        rollerState = RollerState.OFF;
        limitSwitch = new DigitalInput(ArmIntakeConstants.limitSwitchChannel);

        armEncoder.setPosition(0);
    }
    @Override
    public void periodic(){
        System.out.println("Arm Limit Switch: " + limitSwitch.get());

        if (limitSwitch.get()) {
            armEncoder.setPosition(ArmIntakeConstants.armLowSetpoint);
        }

        switch(armState) {
            case HIGH:
                setArmMotor(runArmPID(getArmSparkPosition(), ArmIntakeConstants.armHighSetpoint));
                break;
            case LOW:
                setArmMotor(runArmPID(getArmSparkPosition(), ArmIntakeConstants.armLowSetpoint));
                break;
            case LIMIT:
                setArmMotor(runArmPID(getArmSparkPosition(), ArmIntakeConstants.armLimitSetpoint));
                break;
            case OFF:
                setArmMotor(0);
                break;
        }
        switch(rollerState) {
            case FORWARD:
                setRollerMotor(ArmIntakeConstants.rollerForwardSpeed);
                break;
            case BACKWARD:
                setRollerMotor(ArmIntakeConstants.rollerBackwardSpeed);
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