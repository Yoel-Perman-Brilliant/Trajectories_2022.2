package frc.robot.commands;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.OdometryDrivetrain;

import java.io.IOException;
import java.nio.file.Path;

public class FollowTrajectory extends CommandBase {
    private final OdometryDrivetrain drivetrain;
    private Trajectory trajectory;
    private RamseteCommand ramseteCommand;
    private final RamseteController controller;
    private DifferentialDriveKinematics kinematics;

    public FollowTrajectory(OdometryDrivetrain drivetrain, Trajectory trajectory, PIDSettings leftPIDSettings,
                            PIDSettings rightPIDSettings, FeedForwardSettings feedForwardSettings) {
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
        controller = new RamseteController();
        kinematics = new DifferentialDriveKinematics(drivetrain.getWidth());
        ramseteCommand = new RamseteCommand(trajectory, drivetrain::getPose, controller,
                new SimpleMotorFeedforward(feedForwardSettings.getkS(), feedForwardSettings.getkV(), feedForwardSettings.getkA()), kinematics,
                drivetrain::getWheelSpeeds, new PIDController(leftPIDSettings.getkP(), leftPIDSettings.getkI(), leftPIDSettings.getkD()),
                new PIDController(rightPIDSettings.getkP(), rightPIDSettings.getkI(), rightPIDSettings.getkD()), drivetrain::tankDriveVolts, drivetrain);
    }

    public FollowTrajectory(OdometryDrivetrain drivetrain, String file, PIDSettings leftPIDSettings,
                            PIDSettings rightPIDSettings, FeedForwardSettings feedForwardSettings) {

        this.drivetrain = drivetrain;
        controller = new RamseteController();
        kinematics = new DifferentialDriveKinematics(drivetrain.getWidth());
        Path path = Path.of(Filesystem.getDeployDirectory().getPath(), file);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException e) {
            System.out.println("no trajectory, this is the error");
            e.printStackTrace();
        }
        ramseteCommand = new RamseteCommand(trajectory, drivetrain::getPose, controller,
                new SimpleMotorFeedforward(feedForwardSettings.getkS(), feedForwardSettings.getkV(), feedForwardSettings.getkA()), kinematics,
                drivetrain::getWheelSpeeds, new PIDController(leftPIDSettings.getkP(), leftPIDSettings.getkI(), leftPIDSettings.getkD()),
                new PIDController(rightPIDSettings.getkP(), rightPIDSettings.getkI(), rightPIDSettings.getkD()), drivetrain::tankDriveVolts, drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetOdometry(trajectory.getInitialPose());
        ramseteCommand.initialize();
    }

    @Override
    public void execute() {
        ramseteCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        ramseteCommand.end(interrupted);
        drivetrain.tankDriveVolts(0, 0);
    }

    @Override
    public boolean isFinished() {
        return ramseteCommand.isFinished();
    }
}
