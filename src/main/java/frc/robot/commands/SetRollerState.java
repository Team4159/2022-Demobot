package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmIntake;
import frc.robot.subsystems.ArmIntake.RollerState;

public class SetRollerState extends CommandBase {
    private ArmIntake armIntake;
    private RollerState rollerState;

    public SetRollerState(ArmIntake m_aArmIntake, RollerState m_rollerState) {
        armIntake = m_aArmIntake;
        rollerState = m_rollerState;

        addRequirements(m_aArmIntake);
    }

    @Override
    public void initialize() {
        armIntake.changeRollerState(rollerState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}