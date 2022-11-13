
package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TaxiAuto extends SequentialCommandGroup {
    private Trajectory trajectory = null;
    {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/Default1.wpilib.json");
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory", e.getStackTrace());
        }
    }

    public TaxiAuto(Swerve s_Swerve) {
        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand
        ));
        // addCommands(
        //         new SequentialCommandGroup(
        //             new SetArmState(s_armIntake, ArmState.LOW),
        //             new SetArmState(s_armIntake, ArmState.HIGH),
        //             new SetRollerState(s_armIntake, RollerState.BACKWARD).withTimeout(4),
        //             new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
        //             swerveControllerCommand)); 
                    // new SetArmState(s_armIntake, ArmState.HIGH).withTimeout(2));
                    // new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
                    // swerveControllerCommand).withTimeout(4)
                
                // new SetArmState(s_armIntake, ArmState.HIGH),
                // // new SetRollerState(s_armIntake, RollerState.BACKWARD).withTimeout(4), 
                // new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
                // swerveControllerCommand);
    }
}