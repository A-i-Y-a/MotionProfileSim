import java.lang.Math;

/**
 * <h1>PathGenerator</h1> Generates a motion-profiled pure pursuit path given a
 * path and some robot parameters. The path is a 2-dimensional array (n rows by
 * 2 columns), with each point having an x and y coordinate. <b>UNITS ARE IN
 * INCHES.</b>
 * 
 * @author Allen Du
 * @since 2019-03-10
 */
public class PathGenerator {
	public double[][] path;
	public double pathMaxVel;
	public double accel;
	public double[] segV;
	public double[][] robotPos;
	public double[][] currentLookaheadPoint;
	// Why is this 12 when the recommended distance between points is 6?
	public double lookaheadRadius;
	public int prevClosestPoint;
	public double robotAngle;

	/**
	 * The constructor for a {@code PathGenerator} object.
	 * <b>UNITS ARE IN INCHES.</b>
	 * 
	 * @param route The path, represented by an array of (x, y) coordinates.
	 * @param pathMaxVel The maxmimum theoretical velocity the path can be.
	 * @param accel The acceleration of the robot.
	 * @param lookaheadRadius The radius for the lookahead point. Usually a value from 12-25 inches.
	 */
	public PathGenerator(double[][] route, double pathMaxVel, double accel, double lookaheadRadius) {
		// Doesn't have to be 6, but it is 6.
		// May change the second and third values depending on the path.
		this.path = route;
		
		this.pathMaxVel = pathMaxVel;
		this.accel = accel;
		this.robotPos = new double[][] {
			{0.0, 0.0},
		};
		this.robotAngle = 0.0;
		this.lookaheadRadius = lookaheadRadius;
		// The first lookahead point is just a point along the first. I feel like this is wrong.
		this.currentLookaheadPoint = new double[][] {
			{this.lookaheadRadius * (this.path[1][0] - this.path[0][0]), this.lookaheadRadius * (this.path[1][1] - this.path[0][1])},
			{0.001, 0.0},
		};
		this.prevClosestPoint = 0;

		this.segV = new double[this.path.length];
		this.segV[this.path.length - 1] = 0.0;
		this.segV[0] = this.pathMaxVel;
		for(int i = this.path.length - 2; i >= 1; i--) {
			double[][] temp2 = new double[][] {
				{this.path[i - 1][0], this.path[i - 1][1]},
				{this.path[i][0], this.path[i][1]},
				{this.path[i + 1][0], this.path[i + 1][1]},
			};
			// System.out.println("temp2 value at waypoint " + i + ": " + "\n" + "{" + temp2[0][0] + ", " + temp2[0][1] + "}, \n" + "{" + temp2[1][0] + ", " + temp2[1][1] + "}, \n" + "{" + temp2[2][0] + ", " + temp2[2][1] + "};");
			double kinematicThing = this.segV[i + 1] * this.segV[i + 1] + 2 * this.accel * Magnitude(this.path[i + 1][0], this.path[i + 1][1], this.path[i][0], this.path[i][1]);
			// System.out.println("Kinematic thing at waypoint " + i + ": " + kinematicThing);
			if(kinematicThing < 0) {
				this.segV[i] = maxVelocity(temp2);
			} else {
				this.segV[i] = Math.min(maxVelocity(temp2), kinematicThing);
			}
		}
	}
		
		/**
		 * Distance formula, given two points (x<sub>1</sub>, y<sub>1</sub>) and (x<sub>2</sub>, y<sub>2</sub>).
		 * 
		 * @param x1 
		 * @param y1
		 * @param x2
		 * @param y2
		 * 
		 * @return The distance.
		 */
		public static double Magnitude(double x1, double y1, double x2, double y2) {
			return Math.hypot(x1 - x2, y1 - y2);
		}

		/**
		 * Calculates the curvature of a point.
		 * 
		 * @param path The path.
		 * 
		 * @return The curvature of a given point. 
		 */
		public static double curvature(double[][] path) {
			if(path[0][0] == path[1][0]) {
				path[0][0] += 0.001;
			}
			
			double k1, k2;
			k1 = 0.5 * (path[0][0] * path[0][0] + path[0][1] * path[0][1] - path[1][0] * path[1][0] - path[1][1] * path[1][1]) / (path[0][0] - path[1][0]);
			k2 = (path[0][1] - path[1][1]) / (path[0][0] - path[1][0]);
			double a, b;
			b = 0.5 * (path[1][0] * path[1][0] - 2 * path[1][0] * k1 + path[1][1] * path[1][1] - path[2][0] * path[2][0] + 2 * path[2][0] * k1 - path[2][1] * path[2][1]) / (path[2][0] * k2 - path[2][1] + path[1][1] - path[1][0] * k2);
			a = k1 - k2 * b;
	
			// System.out.println("k1 = " + k1);
			// System.out.println("k2 = " + k2);
			// System.out.println("a = " + a);
			// System.out.println("b = " + b);
	
			double r;
				r = Math.sqrt((path[0][0] - a) * (path[0][0] - a) + (path[0][1] - b) * (path[0][1] - b));

			// System.out.println("Radius: " + r);
	
			// if((path[1][1] - path[0][1]) / (path[1][0] - path[0][0]) == (path[2][1] - path[1][1]) / (path[2][0] - path[1][0])) {
			// 	return 0.0;
			// }

			return 1 / r;
		}

		/**
		 * Calculates the maximum theoretical velocity of a point.
		 * 
		 * @param pathMaxVel The maximum path velocity.
		 * @param path 
		 * 
		 * @return The maximum theoretical velocity of a point.
		 */
		public double maxVelocity(double[][] path) {
			double result = 0.0;
				if((path[1][1] - path[0][1]) / (path[1][0] - path[0][0]) == (path[2][1] - path[1][1]) / (path[2][0] - path[1][0])) {
					// For collinear points
					result = this.pathMaxVel;
				} else {
					// Might change that 2.5 to something else between 1 and 5
					result = Math.min(this.pathMaxVel, 2.5 / curvature(path));
				};
			// System.out.println("Max velocity: " + result);
	
			return result;
		}

		/**
		 * Gives the closest path point to the robot.
		 * 
		 * @return The index of the closest path point.
		 */
		public int closestPoint() {
			// The 0 entry is the distance; the 1 entry is the index.
			double[][] distances = new double[this.path.length - this.prevClosestPoint][2];
			// Just to make sure our robot never goes backwards
			for(int i = this.prevClosestPoint; i < this.path.length; ++i) {
				distances[i - this.prevClosestPoint][0] = Magnitude(this.robotPos[0][0], this.robotPos[0][1], this.path[i][0], this.path[i][1]);
				distances[i - this.prevClosestPoint][1] = (double) i;
				// System.out.println("Distance from current position to point " + distances[i - this.prevClosestPoint][1] + ": " + distances[i - this.prevClosestPoint][0]);
			}
			// Sort the distances in ascending order, then return the index of the shortest distance.
			quickSort(distances, 0, distances.length - 1);
			this.prevClosestPoint = (int) distances[0][1];
			// As to not return the endpoint, which has a target velocity of 0.
			if(this.prevClosestPoint == this.path.length - 1) {
				this.prevClosestPoint--;
			}

			return this.prevClosestPoint;
		}

		/**
		 * Gets you the lookahead point.
		 * 
		 * @return A 2*2 array, with the first row having the x and y coordinates and the second having the fractional index and the index.
		 */
		public double[][] lookaheadPoint(double objectLookaheadRadius) {
            double[][] point;
            double[][] result = this.currentLookaheadPoint;
			point = new double[][] {
				{this.currentLookaheadPoint[0][0], this.currentLookaheadPoint[0][1]},
				// fracIndex, index
				{this.currentLookaheadPoint[1][0], this.currentLookaheadPoint[1][1]},
			};
			int index = (int) Math.round(this.currentLookaheadPoint[1][1]);
			// Another search.
			for(int i = index; i < this.path.length - 1; ++i) {
				double[][] d = new double[][] {
					{this.path[i + 1][0] - this.path[i][0], this.path[i + 1][1] - this.path[i][1]},
				};
				double[][] f = new double[][] {
					{this.path[i][0] - this.robotPos[0][0], this.path[i][1] - this.robotPos[0][1]},
				};
				double a = Magnitude(d[0][0], d[0][1], 0.0, 0.0) * Magnitude(d[0][0], d[0][1], 0.0, 0.0);
				double b = 2 * (d[0][0] * f[0][0] + d[0][1] * f[0][1]);
				double c = Magnitude(f[0][0], f[0][1], 0.0, 0.0) * Magnitude(f[0][0], f[0][1], 0.0, 0.0) - objectLookaheadRadius * objectLookaheadRadius;
				double discriminant = b * b - 4 * a * c;

				if(discriminant < 0 ) {
					
				} else {
					discriminant = Math.sqrt(discriminant);
					double t1 = (-b - discriminant) / (2 * a);
					double t2 = (-b + discriminant) / (2 * a);

					if(t1 >= 0.0 && t1 <= 1.0) {
						t2 = 0;
					} else if(t2 >= 0.0 && t2 <= 1.0) {
						t1 = 0;
					} else {
                        continue;
                    }
					
					point[0][0] = this.path[i][0] + (t1 + t2) * d[0][0];
					point[0][1] = this.path[i][1] + (t1 + t2) * d[0][1];
					point[1][0] = t1 + t2 + i;
					if(point[1][0] > this.currentLookaheadPoint[1][1]) {
						point[1][1] = i;
                        this.currentLookaheadPoint = point;
                        result = point;

						return result;
					}
				}
            };
            result = point;

			return result;
		}
		
		/**
		 * Calculates the curvature of the lookahead point.
		 * 
		 * @return If the value is positive, the lookahead point is on the right; it's on the left if otherwise.
		 */
		public double lookaheadCurvature() {
			updateLookaheadPoint();
			
			double dx = this.currentLookaheadPoint[0][0] - this.robotPos[0][0];
			double dy = this.currentLookaheadPoint[0][1] - this.robotPos[0][1];

			System.out.println("dx and dy: (" + dx + ", " + dy + ")");

			double lookaheadDistance = Magnitude(this.currentLookaheadPoint[0][0], this.currentLookaheadPoint[0][1], this.robotPos[0][0], this.robotPos[0][1]);
			double cos = Math.cos(Math.toRadians(this.robotAngle));
			double tan = Math.tan(Math.toRadians(this.robotAngle));

			// System.out.println("Lookahead distance: " + lookaheadDistance);

			double a = -1.0 * tan;
			double c = tan * this.robotPos[0][0] - this.robotPos[0][1];

			System.out.println("(a, c): (" + a + ", " + c + ")");

			// Point-to-line
			double x = Math.abs(a * this.currentLookaheadPoint[0][0] + this.currentLookaheadPoint[0][1] + c) / Math.sqrt(a * a + 1);

			double sign = Math.signum((this.currentLookaheadPoint[0][0] - this.robotPos[0][0]) - cos * (this.currentLookaheadPoint[0][1] - this.robotPos[0][1]));

			return 2.0 * sign * x / (lookaheadDistance * lookaheadDistance);
		}

		/**
		 * Returns velocities.
		 * 
		 * @param trackWidth The distance between the left and right wheels.
		 * @param robotPos
		 * 
		 * @return The first array entry is the left; the second is the right.
		 */
		public double velocity(double trackWidth, boolean left) {
			double speed = 0.0;
			double c = lookaheadCurvature();
			double v = segV[closestPoint()];
			if(left) {
				speed = v * (2 + c * trackWidth) / 2;
			} else {
				speed = v * (2 - c * trackWidth) / 2;
			};
	
			return speed;
		}

		/**
		 * Updates the robot position.
		 * 
		 * @param xCoord
		 * @param yCoord
		 */
		public void updatePos(double xCoord, double yCoord, double angle) {
			this.robotPos[0][0] = xCoord;
			this.robotPos[0][1] = yCoord;
			this.robotAngle = angle;
			this.robotAngle = this.robotAngle % 360.0;
		}

		public void updateLookaheadPoint() {
			this.currentLookaheadPoint = lookaheadPoint(this.lookaheadRadius);
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
	
	/**
	 * @return The path.
	 */
	public double[][] getPath() {
		return this.path;
	}

	/**
	 * Update the path generator's robot position.
	 * 
	 * @param robotPos
	 */
	public void updatePos(double[][] robotPos, double angle) {
		this.robotPos[0][0] = robotPos[0][0];
		this.robotPos[0][1] = robotPos[0][1];
		angle = angle % 360.0;
		this.robotAngle = angle;
	}
}