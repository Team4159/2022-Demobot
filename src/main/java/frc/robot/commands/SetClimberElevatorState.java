package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Climber.ArmState;
import frc.robot.subsystems.Climber.ElevatorState;

public class SetClimberElevatorState extends CommandBase {
    private ElevatorState elevatorState;
    private Climber s_Climber;

    public SetClimberElevatorState(Climber m_climber, ArmState m_armState, ElevatorState m_eElevatorState) {
        s_Climber = m_climber;
        elevatorState = m_eElevatorState;

        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        //climber.setArmState(armState);
        s_Climber.setElevatorState(elevatorState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
