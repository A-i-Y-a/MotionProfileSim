import java.lang.Math;

public class Robot {
    public double[][] robotPos;

    public Robot(double[][] position) {
        this.robotPos = position;
    }

    public void updatePos(double distance, double angle) {
        this.robotPos[0][0] += distance * Math.cos(Math.toRadians(angle));
        this.robotPos[0][1] += distance * Math.sin(Math.toRadians(angle));
    }

    // TODO add tankdrive to this. Help I actually don't know how to do this.
    public void tankDrive(double lVelocity, double rVelocity) {

    }
}