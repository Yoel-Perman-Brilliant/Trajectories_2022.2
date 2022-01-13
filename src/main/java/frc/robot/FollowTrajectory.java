package frc.robot;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TrajectoryDrivetrain;

import java.io.IOException;
import java.nio.file.Path;

/**
 * The command which allows the robot to follow a trajectory is called RamseteCommand. This is a utility class
 * which supplies the command with the parameters which you have no reason to supply by yourself.
 * For more information about RamseteCommand, see <a href=https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/creating-following-trajectory.html>here</a>.
 */

public class FollowTrajectory {

    /**
     * @param drivetrain          An OdometryDrivetrain implementation.
     * @param trajectory          A trajectory which will be created in the Robot class using the <br>
     *                            TrajectoryUtil.fromPathweaverJson(path) method.
     *                            Get the path using <br>
     *                            Path.of(Filesystem.getDeployDirectory().getPath(), "aPath"), where aPath is the name of the Patheweaver
     *                            output file, which you shold put in the src\main\deploy folder.
     * @param leftPIDSettings     PID settings for the left speed controllers.
     * @param rightPIDSettings    PID settings for the right speed controllers.
     * @param feedForwardSettings Feed Forward settings, obviously.
     * @return a ramsete command ready for execution.
     */

    public static RamseteCommand getCommand(OdometryDrivetrain drivetrain, Trajectory trajectory, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                                            FeedForwardSettings feedForwardSettings) {
        RamseteController ramseteController = new RamseteController();
        DifferentialDriveKinematics ddk = new DifferentialDriveKinematics(drivetrain.getWidth());

        return new RamseteCommand(trajectory, drivetrain::getPose, ramseteController,
                new SimpleMotorFeedforward(feedForwardSettings.getkS(), feedForwardSettings.getkV(), feedForwardSettings.getkA()), ddk,
                drivetrain::getWheelSpeeds, new PIDController(leftPIDSettings.getkP(), leftPIDSettings.getkI(), leftPIDSettings.getkD()),
                new PIDController(rightPIDSettings.getkP(), rightPIDSettings.getkI(), rightPIDSettings.getkD()), drivetrain::tankDriveVolts, drivetrain);

    }

    public static RamseteCommand getCommand(OdometryDrivetrain drivetrain, String pathFile, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                                            FeedForwardSettings feedForwardSettings) {
        Path path = Path.of(Filesystem.getDeployDirectory().getPath(), "horizontal.wpilib.json");
        Trajectory trajectory = null;
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException e) {
            System.out.println("no trajectory, this is the error");
            e.printStackTrace();
        }
        return getCommand(drivetrain, trajectory, leftPIDSettings, rightPIDSettings,
                feedForwardSettings);
    }


}
