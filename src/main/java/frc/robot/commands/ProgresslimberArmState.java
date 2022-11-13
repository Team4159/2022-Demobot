package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ArmState;

public class ProgresslimberArmState extends CommandBase {
    private Climber s_Climber;
    private ArmState armState;
    private static Map<ArmState, ArmState> stateProg = Map.of(
        ArmState.HIGH, ArmState.MID,
        ArmState.MID, ArmState.LOW,
        ArmState.LOW, ArmState.MID,
        ArmState.DOWN, ArmState.HIGH
    );

    public ProgresslimberArmState(Climber m_climber) {
        s_Climber = m_climber;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        if (s_Climber.getActualArmState().equals(ArmState.OFF)) return;
        armState = stateProg.get(s_Climber.getDesiredArmState());
        s_Climber.setArmState(armState);
    }

    @Override
    public boolean isFinished() {
        return s_Climber.getActualArmState().equals(armState);
    }

    @Override
    public void end(boolean i) {
        super.end(i);
        if (i) s_Climber.setArmState(ArmState.OFF);
    }
}
