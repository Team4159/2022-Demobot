package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ArmState;

public class ResetClimberArmState extends CommandBase {
    private Climber climber;

    public ResetClimberArmState(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setArmState(ArmState.DOWN);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
