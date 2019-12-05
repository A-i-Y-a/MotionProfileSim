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
        this.xCoeffs[0] = this.start[0][0];
        this.xCoeffs[1] = this.start[1][0];
        this.xCoeffs[2] = 0.5 * this.start[2][0];
        this.xCoeffs[3] = (10 * (this.end[0][0] - this.start[0][0]) - (4 * this.end[1][0] + 6 * this.start[1][0]) - (1.5 * this.start[2][0] - 0.5 * this.end[2][0]));
        this.xCoeffs[4] = (15 * (this.end[0][0] - this.start[0][0]) + 7 * this.end[1][0] + 8 * this.start[1][0] + 1.5 * this.start[2][0] - this.end[2][0]);
        this.xCoeffs[5] = (6 * (this.end[0][0] - this.start[0][0]) - 3 * (this.end[1][0] + this.start[1][0]) + 0.5 * (this.end[2][0] - this.start[2][0]));

        this.yCoeffs[0] = this.start[0][1];
        this.yCoeffs[1] = this.start[1][1];
        this.yCoeffs[2] = 0.5 * this.start[2][1];
        this.yCoeffs[3] = (10 * (this.end[0][1] - this.start[0][1]) - (4 * this.end[1][1] + 6 * this.start[1][1]) - (1.5 * this.start[2][1] - 0.5 * this.end[2][1]));
        this.yCoeffs[4] = (15 * (this.end[0][1] - this.start[0][1]) + 7 * this.end[1][1] + 8 * this.start[1][1] + 1.5 * this.start[2][1] - this.end[2][1]);
        this.yCoeffs[5] = (6 * (this.end[0][1] - this.start[0][1]) - 3 * (this.end[1][1] + this.start[1][1]) + 0.5 * (this.end[2][1] - this.start[2][1]));

        double xCoeffsSum = 0.0;
        double yCoeffsSum = 0.0;

        for(int i = 0; i < 6; ++i) {
            xCoeffsSum += this.xCoeffs[i];
            yCoeffsSum += this.yCoeffs[i];

        }

        for(int j = 0; j < 6; ++j) {
            this.xCoeffs[j] = this.xCoeffs[j] * this.end[0][0] / xCoeffsSum;
            this.yCoeffs[j] = this.yCoeffs[j] * this.end[0][1] / yCoeffsSum;
        }
    }

    /** 
     * Gets the position of a point along the spline.
     * 
     * @param t The proportion of the path done at that point. Range from 0 to 1.
     * 
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
     * 
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
     * 
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
     * 
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
     * Arc length, but from one point to another point.
     * 
     * @param interval The interval to increment the trapezoidal sum by.
     * @param start The start point.
     * @param end The end point.
     * 
     * @return The arc length.
     */
    public double arcLength(double interval, double start, double end) {
        double result = 0.0;
        double increment = start;
        while(increment <= end) {
            double derivatives1[][] = getVelocity(increment);
            increment += interval;
            double derivatives2[][] = getVelocity(increment);
            result += 0.5 * interval * (Math.hypot(derivatives1[0][0], derivatives1[0][1]) + Math.hypot(derivatives2[0][0], derivatives2[0][1]));
        }

        return result;
    }

    // // I keep getting Stack Overflow errors HELP
    // public double arcLengthAt(int index, int numPoints) {
    //     if(index == 0) {
    //         return 0.0;
    //     } else {
    //         return cumArcLengthAt(index, numPoints, 0.0);
    //     }
    // }

    // // YES I FINALLY FIXED STACK OVERFLOW
    // public double cumArcLengthAt(int index, int numPoints, double cumArcLength) {
    //     if(index == 0) {
    //         return 0.0;
    //     } else {
    //         cumArcLength += arcLength(0.001, (1.0 * index - 1.0) / (1.0 * numPoints), 1.0 * index / (1.0 * numPoints));
    //         return cumArcLengthAt(index - 1, numPoints, cumArcLength);
    //     }
    // }

    // YES I FIXED STACK OVERFLOW
    /**
     * Spline interpolation. The start and endpoints will always be included.
     * 
     * @param interval See {@code arcLength}.
     * @param spacing The space, in inches, between each point. Doesn't really work.
     * 
     * @return An n*5 array of (x, y, xVel, yVel, arcLength) coordinates of interpolated points. 
     */
    public double[][] interpolate(double interval, double spacing) {
        double arc = arcLength(interval);
        int numPoints = (int) Math.floor(arc / spacing);
        double[][] result = new double[numPoints + 1][5];
        result[0][4] = 0.0;
        for(int i = 0; i <= numPoints; ++i) {
            double[][] temp = getPosition((1.0 * i) / numPoints);
            double[][] temp2 = getVelocity((1.0 * i) / numPoints);

            result[i][0] = temp[0][0];
            result[i][1] = temp[0][1];
            result[i][2] = temp2[0][0];
            result[i][3] = temp2[0][1];
            if(i != 0) {
                result[i][4] = arcLength(interval, (1.0 * i - 1.0) / numPoints, (1.0 * i) / numPoints) + result[i - 1][4];
            }
        }

        return result;
    }

    /**
     * I cannot figure out to interpolate with even space a quintic spline so here's something that's really jank.
     * 
     * @param interval See {@code arcLength}.
     * @param spacing The space, in inches, between each point.
     * 
     * @return An evenly spaced interpolated spline.
     */
    public double[][] interpolateEven(double interval, double spacing) {
        double[][] jankPath = interpolate(interval, 0.001);
        double[] jankPathArcLength = new double[jankPath.length];
        for(int k = 0; k < jankPath.length; ++k) {
            jankPathArcLength[k] = jankPath[k][4];
        }
        double arc = arcLength(interval);
        int numPoints = (int) Math.floor(arc / spacing);
        double[][] result = new double[numPoints + 1][4];
        int[] closestIndices = new int[numPoints];
        closestIndices[0] = 0;
        for(int i = 0; i < 4; ++i) {
            result[0][i] = jankPath[0][i];
            result[numPoints][i] = jankPath[jankPath.length - 1][i];
        }
        for(int j = 1; j < numPoints; ++j) {
            closestIndices[j] = closestElement(jankPathArcLength, 6.0 * j);
            // System.out.println("Index " + j + ": " + closestIndices[j]);
        }
        for(int l = 0; l < numPoints; ++l) {
            for(int m = 0; m < 4; ++m) {
                result[l][m] = jankPath[closestIndices[l]][m];
            }
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

    /**
     * The value in the set that is the closest to and less than the desired value.
     * 
     * @param array The array. 
     * @param value The value.
     * 
     * @return The index in the array such that {@code array[j]} is the closest to and less than the value.
     */
    public static int closestElement(double[] array, double value) {
        double[][] differences = new double[array.length][2];
        double[][] absDifferences = new double[differences.length][2];
        for(int i = 0; i < differences.length; ++i) {
            differences[i][1] = i;
            absDifferences[i][1] = i;
            differences[i][0] = array[i] - value;
            absDifferences[i][0] = Math.abs(differences[i][0]);

            // System.out.println("Difference for index " + i + ": " + differences[i][0]);
        }
        quickSort(absDifferences, 0, differences.length - 1);
        int j = 0;
        while(differences[j][0] < 0 && j < differences.length - 1) {
            j++;
        }

        return j;
    }

        /**
		 * I copied the quicksort sorting algorithm and adjusted it for 2-dimensional arrays.
		 */
		public static void quickSort(double[][] arr, int low, int high)
        {
            //check for empty or null array
            if (arr == null || arr.length == 0){
                return;
            }
             
            if (low >= high){
                return;
            }
     
            //Get the pivot element from the middle of the list
            int middle = low + (int) Math.round((high - low) / 2);
            double pivot = arr[middle][0];
     
            // make left < pivot and right > pivot
            int i = low, j = high;
            while (i <= j)
            {
                //Check until all values on left side array are lower than pivot
                while (arr[i][0] < pivot)
                {
                    i++;
                }
                //Check until all values on left side array are greater than pivot
                while (arr[j][0] > pivot)
                {
                    j--;
                }
                //Now compare values from both side of lists to see if they need swapping
                //After swapping move the iterator on both lists
                if (i <= j)
                {
                    swap (arr, i, j);
                    i++;
                    j--;
                }
            }
            //Do same operation as above recursively to sort two sub arrays
            if (low < j){
                quickSort(arr, low, j);
            }
            if (high > i){
                quickSort(arr, i, high);
            }
        }
         
        /**
         * Swapping two variables.
         */
        public static void swap (double[][] array, int x, int y)
        {
            double temp = array[x][0];
            double temp2 = array[x][1];
            array[x][0] = array[y][0];
            array[x][1] = array[y][1];
            array[y][0] = temp;
            array[y][1] = temp2;
        }
 }