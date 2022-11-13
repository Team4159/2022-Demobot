package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.RollerState;

public class SetRollerState extends CommandBase {
    private Intake armIntake;
    private RollerState rollerState;

    public SetRollerState(Intake m_aArmIntake, RollerState m_rollerState) {
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

    @Override
    public void end(boolean i) {
        armIntake.changeRollerState(RollerState.OFF);
    }
}