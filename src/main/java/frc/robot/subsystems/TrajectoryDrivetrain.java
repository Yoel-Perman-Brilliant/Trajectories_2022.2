package frc.robot.subsystems;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.OdometryDrivetrain;
import frc.robot.Robot;

public class TrajectoryDrivetrain extends OdometryDrivetrain {


    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final ADXRS450_Gyro gyro;
    private final DifferentialDriveOdometry odometry;
    private final Field2d field2d;

    public TrajectoryDrivetrain(MotorControllerGroup leftSCG, MotorControllerGroup rightSCG,
                                Encoder leftEncoder, Encoder rightEncoder, ADXRS450_Gyro gyro) {
        super(leftSCG, rightSCG);
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.gyro = gyro;

        this.leftEncoder.reset();
        this.rightEncoder.reset();
        this.gyro.reset();

        this.odometry = new DifferentialDriveOdometry(rotFromAngle());

        this.field2d = new Field2d();
        Robot.root.putData("Field 2d", field2d);
        Robot.root.putNumber("gyro value", gyro::getAngle);
        Robot.root.putNumber("left encoder", leftEncoder::getDistance);
        Robot.root.putNumber("right encoder", rightEncoder::getDistance);
        Robot.root.putNumber("x", () -> odometry.getPoseMeters().getX());
        Robot.root.putNumber("y", () -> odometry.getPoseMeters().getY());

    }

    @Override
    public void periodic() {
        odometry.update(rotFromAngle(), leftEncoder.getDistance(), rightEncoder.getDistance());
        field2d.setRobotPose(odometry.getPoseMeters());
        Robot.root.update();

    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        leftEncoder.reset();
        rightEncoder.reset();
        gyro.reset();
        odometry.resetPosition(pose, rotFromAngle());
    }

    public Rotation2d rotFromAngle() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftController.setVoltage(leftVolts);
        rightController.setVoltage(rightVolts);
    }

    public double getWidth() {
        return 0.65;
    }
}
