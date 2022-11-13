package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ArmState;

public class ProgressClimberArmState extends CommandBase {
    private Climber climber;
    private ArmState armState;
    private static Map<ArmState, ArmState> stateProg = Map.of(
        ArmState.HIGH, ArmState.MID,
        ArmState.MID, ArmState.LOW,
        ArmState.LOW, ArmState.MID,
        ArmState.DOWN, ArmState.HIGH,
        ArmState.OFF, ArmState.DOWN
    );

    public ProgressClimberArmState(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        armState = stateProg.get(climber.getDesiredArmState());
        climber.setArmState(armState);
    }

    @Override
    public boolean isFinished() {
        return climber.getActualArmState().equals(armState);
    }

    @Override
    public void end(boolean i) {
        if (i) climber.setArmState(ArmState.OFF);
    }
}
