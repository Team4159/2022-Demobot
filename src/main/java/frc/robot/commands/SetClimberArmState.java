package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ArmState;

public class SetClimberArmState extends CommandBase {
    private Climber s_Climber;
    private ArmState armState;

    public SetClimberArmState(Climber m_climber, ArmState m_armState) {
        s_Climber = m_climber;

        armState = m_armState;

        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        s_Climber.setArmState(armState);
        //s_Climber.toggleArm();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
