package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmIntake;
import frc.robot.subsystems.ArmIntake.ArmState;

public class SetArmState extends CommandBase {
    
    private ArmIntake armIntake;
    private ArmState armState;

    public SetArmState(ArmIntake m_armIntake, ArmState m_armState) {
        armState = m_armState;
        armIntake = m_armIntake;

        addRequirements(m_armIntake);
    }

    @Override
    public void initialize() {
        armIntake.changeArmState(armState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
