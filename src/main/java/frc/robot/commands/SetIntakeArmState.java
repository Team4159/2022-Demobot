package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.ArmState;

public class SetIntakeArmState extends CommandBase {
    
    private Intake intake;
    private ArmState armState;

    public SetIntakeArmState(Intake m_armIntake, ArmState m_armState) {
        armState = m_armState;
        intake = m_armIntake;

        addRequirements(m_armIntake);
    }

    @Override
    public void initialize() {
        intake.setDesiredArmState(armState);
    }

    @Override
    public boolean isFinished() {
        return intake.getActualArmState().equals(armState);
    }
}
