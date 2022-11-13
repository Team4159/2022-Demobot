package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;

public class DefenseModeCommand extends CommandBase {
    
    private frc.robot.subsystems.Swerve s_swerve;
    private boolean state;

    public DefenseModeCommand(Swerve swerve, boolean m_state) { //true is defense mode, false is regular driving
        s_swerve = swerve;
        state = m_state;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        s_swerve.setDefenseMode(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
