import java.lang.Math;

public class Robot {
    public double[][] robotPos;
    public double robotAngle;
    public double timeInterval;
    public double trackLength;

    /**
     * Constructor.
     * 
     * @param position Initial robot position coordinates. For robot-centric, this is (0, 0).
     * @param time Refresh rate of the RoboRIO. Or somthing else. Electronics is hard.
     * @param trackLength Width of the drivetrain.
     */
    public Robot(double[][] position, double time, double trackLength) {
        this.robotPos = position;
        this.robotAngle = 0.0;
        this.timeInterval = 0.02;
        this.trackLength = trackLength;
    }

    /**
     * Updates the robot position.
     * 
     * @param polar If we're updating the robot based on magnitude and direction of displacement vector or displacement vector components.
     * @param distance Either the magnitude or the x-component of the displacement vector.
     * @param angle Either the direction, in degrees, or the y-component of the displacement vector.
     */
    public void updatePos(Boolean polar, double distance, double angle) {
        if(polar) {
            this.robotPos[0][0] += distance * Math.cos(Math.toRadians(angle));
            this.robotPos[0][1] += distance * Math.sin(Math.toRadians(angle));
            updateAngle(angle);
        } else {
            this.robotPos[0][0] += distance;
            this.robotPos[0][1] += angle;
            updateAngle();
        }
    }

    // TODO Add rotational inertia to this. HELP DOUBLE INTEGRATION IS HARD.
    /**
     * Tank drive. This is the same way you drive remote control cars--one side controls the velocity of the left wheels, and the other side controls the velocity of the rigth wheels.
     * Uses the physics of uniform circular motion. 
     * 
     * @param lVelocity Velocity of the left wheels (motor control group).
     * @param rVelocity Velocity of the right wheels (motor control group).
     */
    public void tankDrive(double lVelocity, double rVelocity) {
        // System.out.println("Left = Right? " + (lVelocity == rVelocity));
        if(lVelocity == rVelocity) {
            this.robotPos[0][0] += lVelocity * Math.cos(Math.toRadians(this.robotAngle)) * this.timeInterval;
            this.robotPos[0][1] += lVelocity * Math.sin(Math.toRadians(this.robotAngle)) * this.timeInterval;
        } else if(Math.abs(lVelocity) >= Math.abs(rVelocity)) {
            if(lVelocity >= 0) {
                if(rVelocity >= 0) {
                    double exradius = 0.5 * this.trackLength + rVelocity * this.trackLength / (lVelocity - rVelocity);
                    double[][] projectedVector = new double[][] {
                        {exradius * Math.cos(Math.toRadians(this.robotAngle)), exradius * Math.sin(Math.toRadians(this.robotAngle))}
                    };
                    double angularVelocity = (lVelocity - rVelocity) / this.trackLength;
                    projectedVector = rotate(projectedVector, -1.0 * angularVelocity * this.timeInterval);
                    this.robotPos[0][0] += projectedVector[0][0];
                    this.robotPos[0][1] += projectedVector[0][1];
                } else {
                    double inradius = 0.5 * this.trackLength + rVelocity * this.trackLength / (lVelocity - rVelocity);
                    double[][] projectedVector = new double[][] {
                        {inradius * Math.cos(Math.toRadians(this.robotAngle)), inradius * Math.sin(Math.toRadians(this.robotAngle))}
                    };
                    double angularVelocity = (lVelocity - rVelocity) / this.trackLength; 
                    projectedVector = rotate(projectedVector, -1.0 * angularVelocity * this.timeInterval);
                    this.robotPos[0][0] += projectedVector[0][0];
                    this.robotPos[0][1] += projectedVector[0][1];
                }
            } else {
                if(rVelocity >= 0) {
                    double inradius = 0.5 * this.trackLength + rVelocity * this.trackLength / (rVelocity - lVelocity);
                    double[][] projectedVector = new double[][] {
                        {inradius * Math.cos(Math.toRadians(this.robotAngle)), inradius * Math.sin(Math.toRadians(this.robotAngle))}
                    };
                    double angularVelocity = (rVelocity - lVelocity) / this.trackLength;
                    projectedVector = rotate(projectedVector, 1.0 * angularVelocity * this.timeInterval);
                    this.robotPos[0][0] += projectedVector[0][0];
                    this.robotPos[0][1] += projectedVector[0][1];
                } else {
                    double exradius = 0.5 * this.trackLength + rVelocity * this.trackLength / (lVelocity - rVelocity);
                    double[][] projectedVector = new double[][] {
                        {exradius * Math.cos(Math.toRadians(this.robotAngle)), exradius * Math.sin(Math.toRadians(this.robotAngle))}
                    };
                    double angularVelocity = (lVelocity - rVelocity) / this.trackLength;
                    projectedVector = rotate(projectedVector, 1.0 * angularVelocity * this.timeInterval);
                    this.robotPos[0][0] += projectedVector[0][0];
                    this.robotPos[0][1] += projectedVector[0][1];
                }
            }
        } else {
            if(rVelocity >= 0) {
                if(lVelocity >= 0) {
                    double exradius = 0.5 * this.trackLength + this.trackLength * lVelocity / (rVelocity - lVelocity);
                    double[][] projectedVector = new double[][] {
                        {exradius * Math.cos(Math.toRadians(this.robotAngle)), exradius * Math.sin(Math.toRadians(this.robotAngle))}
                    };
                    double angularVelocity = (rVelocity - lVelocity) / this.trackLength;
                    projectedVector = rotate(projectedVector, angularVelocity * this.timeInterval);
                    this.robotPos[0][0] += projectedVector[0][0];
                    this.robotPos[0][1] += projectedVector[0][1];
                } else {
                    double inradius = 0.5 * this.trackLength - this.trackLength * rVelocity / (rVelocity - lVelocity);
                    double[][] projectedVector = new double[][] {
                        {inradius * Math.cos(Math.toRadians(this.robotAngle)), inradius * Math.sin(Math.toRadians(this.robotAngle))}
                    }; 
                    double angularVelocity = (rVelocity - lVelocity) / this.trackLength;
                    projectedVector = rotate(projectedVector, angularVelocity * this.timeInterval);
                    this.robotPos[0][0] += projectedVector[0][0];
                    this.robotPos[0][1] += projectedVector[0][1];
                }
            } else {
                if(lVelocity >= 0) {
                    double inradius = 0.5 * this.trackLength + rVelocity * this.trackLength / (lVelocity - rVelocity);
                    double[][] projectedVector = new double[][] {
                        {inradius * Math.cos(Math.toRadians(this.robotAngle)), inradius * Math.sin(Math.toRadians(this.robotAngle))}
                    }; 
                    double angularVelocity = (lVelocity - rVelocity) / this.trackLength;
                    projectedVector = rotate(projectedVector, -1.0 * angularVelocity * this.timeInterval);
                    this.robotPos[0][0] += projectedVector[0][0];
                    this.robotPos[0][1] += projectedVector[0][1];
                } else {
                    double exradius = 0.5 * this.trackLength + this.trackLength * lVelocity / (rVelocity - lVelocity);
                    double[][] projectedVector = new double[][] {
                        {exradius * Math.cos(Math.toRadians(this.robotAngle)), exradius * Math.sin(Math.toRadians(this.robotAngle))}
                    };
                    double angularVelocity = (rVelocity - lVelocity) / this.trackLength;
                    projectedVector = rotate(projectedVector, -1.0 * angularVelocity * this.timeInterval);
                    this.robotPos[0][0] += projectedVector[0][0];
                    this.robotPos[0][1] += projectedVector[0][1];
                }
            }
        }
    }

    /**
     * Updates the robot angle manually. ROBOT ANGLE IS IN DEGREES.
     * 
     * @param angle The angle to add.
     */
    public void updateAngle(double angle) {
        this.robotAngle += angle;
        this.robotAngle = this.robotAngle % 360.0;
    }

    /**
     * Updates the robot angle based on robot position.
     */
    public void updateAngle() {
        this.robotAngle = Math.toDegrees(Math.atan2(this.robotPos[0][1], this.robotPos[0][0]));
        this.robotAngle = this.robotAngle % 360.0;
    }

    /**
     * Uses the 2*2 rotation matrix for vectors to rotate a vector by a certain angle.
     * 
     * @param vector The vector, in (x, y), to rotate.
     * @param angle The angle, in degrees, to rotate it by.
     */
    public double[][] rotate(double[][] vector, double angle) {
        double[][] result = new double[][] {
            {vector[0][0] * Math.cos(Math.toRadians(angle)) - vector[0][1] * Math.sin(Math.toRadians(angle)), vector[0][0] * Math.sin(Math.toRadians(angle)) + vector[0][1] * Math.cos(Math.toRadians(angle))}
        };

        return result;
    }
}