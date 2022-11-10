package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Climber.ArmState;
import frc.robot.subsystems.Climber.ElevatorState;

public class SetClimberState extends CommandBase {
    
    private ArmState armState;
    private ElevatorState elevatorState;

    private Climber climber;

    public SetClimberState(Climber m_climber, ArmState m_armState, ElevatorState m_eElevatorState) {
        climber = m_climber;
        armState = m_armState;
        elevatorState = m_eElevatorState;

        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        climber.setArmState(armState);
        climber.setElevatorState(elevatorState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
