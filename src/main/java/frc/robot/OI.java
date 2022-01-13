package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
    private Joystick left = new Joystick(0);
    private Joystick right = new Joystick(1);

    public OI() {}

    public double getLeftX() {
        return left.getX()*0.7;
    }

    public double getLeftY() {
        return -left.getY()*0.7;
    }

    public double getRightX() {

        return right.getX()*0.7;
    }

    public double getRightY() {
        return -right.getY()*0.7;
    }
}
