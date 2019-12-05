import java.lang.Math;

/**
 * <h1> Spline </h1>
 * 
 * Generates a quintic spline given the 
 * position, velocity, and acceleration of the start and endpoints.
 * The parameter goes from 0 to 1 and is defined as the proportion of the path done.
 * 
 * <b> UNITS ARE IN INCHES. </b>
 * 
 * @author Allen Du
 * @since 2019-04-11
 */

 public class Spline {
    // Position, velocity, acceleration
    public double[][] start;
    public double[][] end;

    // Index i corresponds to the coefficient of x^i
    public double[] xCoeffs;
    public double[] yCoeffs;

    /**
     * Constructor. 
     * 
     * @param start An array, containing the position, velocity, and acceleration vectors of the start point in (x, y) format, in that order.
     * @param end Same format for the start vectors.
     */
    public Spline(double[][] start, double[][] end) {
        this.start = new double[3][2];
        this.end = new double[3][2];
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 2; ++j) {
                this.start[i][j] = start[i][j];
                this.end[i][j] = end[i][j];
            }

        this.xCoeffs = new double[6];
        this.yCoeffs = new double[6];
        calculateCoeffs();
        }
    }

    /**
     * Calculates the coefficients of the spline.
     */
    public void calculateCoeffs() {
        this.xCoeffs[0] = this.start[0][0] / 31.0;
        this.xCoeffs[1] = this.start[1][0] / 31.0;
        this.xCoeffs[2] = 0.5 * this.start[2][0] / 31.0;
        this.xCoeffs[3] = (10 * (this.end[0][0] - this.start[0][0]) - (4 * this.end[1][0] + 6 * this.start[1][0]) - (1.5 * this.start[2][0] - 0.5 * this.end[2][0])) / 31.0;
        this.xCoeffs[4] = (15 * (this.end[0][0] - this.start[0][0]) + 7 * this.end[1][0] + 8 * this.start[1][0] + 1.5 * this.start[2][0] - this.end[2][0]) / 31.0;
        this.xCoeffs[5] = (6 * (this.end[0][0] - this.start[0][0]) - 3 * (this.end[1][0] + this.start[1][0]) + 0.5 * (this.end[2][0] - this.start[2][0])) / 31.0;

        this.yCoeffs[0] = this.start[0][1] / 31.0;
        this.yCoeffs[1] = this.start[1][1] / 31.0;
        this.yCoeffs[2] = 0.5 * this.start[2][1] / 31.0;
        this.yCoeffs[3] = (10 * (this.end[0][1] - this.start[0][1]) - (4 * this.end[1][1] + 6 * this.start[1][1]) - (1.5 * this.start[2][1] - 0.5 * this.end[2][1])) / 31.0;
        this.yCoeffs[4] = (15 * (this.end[0][1] - this.start[0][1]) + 7 * this.end[1][1] + 8 * this.start[1][1] + 1.5 * this.start[2][1] - this.end[2][1]) / 31.0;
        this.yCoeffs[5] = (6 * (this.end[0][1] - this.start[0][1]) - 3 * (this.end[1][1] + this.start[1][1]) + 0.5 * (this.end[2][1] - this.start[2][1])) / 31.0;
    }

    /** 
     * Gets the position of a point along the spline.
     * 
     * @param t The proportion of the path done at that point. Range from 0 to 1.
     * @return a 1*2 array, in the form (x, y).
     */
    public double[][] getPosition(double t) {
        double[][] result = new double[1][2];
        for(int i = 0; i < 6; ++i) {
            result[0][0] += this.xCoeffs[i] * Math.pow(t, i);
            result[0][1] += this.yCoeffs[i] * Math.pow(t, i);
        }

        return result;
    }

    /**
     * Same format as the position method.
     * 
     * @param t
     * @return The velocity vector at the point.
     */
    public double[][] getVelocity(double t) {
        double[][] result = new double[1][2];
        for(int i = 1; i < 6; ++i) {
            result[0][0] += i * this.xCoeffs[i] * Math.pow(t, i - 1);
            result[0][1] += i * this.yCoeffs[i] * Math.pow(t, i - 1);
        }

        return result;
    }

    /**
     * You get it, right?
     * 
     * @param t
     * @return The acceleration vector at the point.
     */
    public double[][] getAccel(double t) {
        double[][] result = new double[1][2];
        for(int i = 2; i < 6; ++i) {
            result[0][0] += i * (i - 1) * this.xCoeffs[i] * Math.pow(t, i - 2);
            result[0][1] += i * (i - 1) * this.yCoeffs[i] * Math.pow(t, i - 2);
        }

        return result;
    }

    /**
     * Uses a trapezoidal Riemann sum to get the arc length of the spline.
     * 
     * @param interval The interval length.
     * @return The arc length.
     */
    public double arcLength(double interval) {
        double result = 0.0;
        for(int i = 0; i < (int) Math.floor(1 / interval); i++) {
            double derivatives1[][] = getVelocity(i * interval);
            // System.out.println("Velocity at position " + i * interval + ": (" + derivatives1[0][0] + "," + derivatives1[0][1] + ")");
            double derivatives2[][] = getVelocity(interval * (i + 1));
            result = result + 0.5 * interval * (Math.hypot(derivatives1[0][0], derivatives1[0][1]) + Math.hypot(derivatives2[0][0], derivatives2[0][1]));
            // System.out.println("Arc length at position " + i * interval + ": " + result);
            // double integrand = Math.hypot(derivatives1[0][0], derivatives1[0][1]) + Math.hypot(derivatives2[0][0], derivatives2[0][1]);
            // System.out.println("Terms in the calculation: " + integrand);
            // System.out.println("So is the problem the interval? " + interval);
            // double expression = 0.5 * interval * (Math.hypot(derivatives1[0][0], derivatives1[0][1]) + Math.hypot(derivatives2[0][0], derivatives2[0][1]));
            // System.out.println("So what the hell is the problem? " + expression);
            // You're kidding me, right? Really? (1/2) doesn't work but 0.5 does? What in the actual.
        }

        return result;
    }

    /**
     * Spline interpolation. The start and endpoints will always be included.
     * 
     * @param interval See {@code arcLength}.
     * @param spacing The space, in inches, between each point.
     * @return An n*4 array of (x, y, xVel, yVel) coordinates of interpolated points. 
     */
    public double[][] interpolate(double interval, double spacing) {
        double arc = arcLength(interval);
        int numPoints = (int) Math.floor(arc / spacing);
        double[][] result = new double[numPoints + 1][4];
        for(int i = 0; i <= numPoints; ++i) {
            double[][] temp = getPosition((1.0 * i) / numPoints);
            double[][] temp2 = getVelocity((1.0 * i) / numPoints);

            result[i][0] = temp[0][0];
            result[i][1] = temp[0][1];
            result[i][2] = temp2[0][0];
            result[i][3] = temp2[0][1];
        }

        return result;
    }

    /**
     * If you want the robot to end at a certain angle.
     * 
     * @param angle The angle, in degrees, from the zero point.
     * @param velocity The magnitude of the velocity vector of the endpoint.
     * @param acceleration The magnitude of the acceleration vector of the endpoint.
     * 
     * @return A 2*2 array, with the first row being the velocity vector and the next being the acceleration vector.
     */
    public static double[][] givenAngle(double angle, double velocity, double acceleration) {
        double angleRadian = angle * Math.PI / 180.0;
        double[][] result = new double[][] {
            {velocity * Math.cos(angleRadian), velocity * Math.sin(angleRadian)},
            {acceleration * Math.cos(angleRadian), acceleration * Math.sin(angleRadian)}
        };

        return result;
    }

    /**
     * Gets the speed given a parameter. See {@code getVelocity} for the format.
     * 
     * @return The magnitude of the velocity vector.
     */
    public double speed(double t) {
        double[][] temp = getVelocity(t);

        return Math.hypot(temp[0][0], temp[0][1]);
    }
 }