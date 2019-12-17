class Main {

    public static void main(String[] args) {
        double[][] testLineStart = new double[][] {
            {0.0, 0.0},
            {1.0, 1.0},
            {0.0, 0.0},
        };
        double[][] testLineEnd = new double[][] {
            {100.0, 100.0},
            {0.0, 0.0},
            {0.0, 0.0},
        };

        Spline testLineSpline = new Spline(testLineStart, testLineEnd);
        // testLineSpline.calculateCoeffs();
        // for(int i = 0; i < 6; ++i) {
        //     System.out.println("Coefficient of x^" + i + ": " + testLineSpline.xCoeffs[i] + "; coefficient of y^" + i + ": " + testLineSpline.yCoeffs[i]);
        // }
        // for(int j = 0; j < 101; ++j) {
        //     double[][] tempPos = testLineSpline.getPosition(j / 100.0);
        //     System.out.println("Position: " + "(" + tempPos[0][0] + ", " + tempPos[0][1] + ")");
        // }
        // System.out.println("Arc length: " + testLineSpline.arcLength(0.000001));

        // for(int k = 0; k < 24; ++k) {
        //     // System.out.println("Waypoint " + k + " position: (" + testGenerator.path[k][0] + ", " + testGenerator.path[k][1] + ")");

        //     // System.out.println("Waypoint " + k + " velocity: " + testGenerator.segV[k]);
        // }

        // For better accuracy add a 0
        double[][] testSplinePath = testLineSpline.interpolateEven(0.00000001, 6.0);
        PathGenerator testGenerator = new PathGenerator(testSplinePath, 10.0, 2.0, 12.0);


        // double[] testValues = new double[100];
        // for(int l = 1; l < 101; ++l) {
        //     testValues[l - 1] = 1.0 * l;
        // }

        // for(int m = 0; m < testSplinePath.length; ++m) {
        //     System.out.println("Waypoint " + m + " position: (" + testSplinePath[m][0] + ", " + testSplinePath[m][1] + ")");
        // }

        // double[][] testPos = {
        //     {50.0, 50.0},
        // };
        // Robot testBot = new Robot(testPos);
        // testGenerator.updatePos(testPos);
        // System.out.println("Position: (" + testGenerator.robotPos[0][0] + ", " + testGenerator.robotPos[0][1] + ")");
        // System.out.println("Closest point: " + testGenerator.closestPoint());

        // double[][] testSort = new double[100][2];
        // for(int l = 0; l < 100; ++l) {
        //     testSort[l][0] = (double) 100 - l;
        //     testSort[l][1] = (double) l;
        // }
        // PathGenerator.quickSort(testSort, 0, testSort.length - 1);
        // System.out.println("testSort first element: (" + testSort[0][0] + ", " + testSort[0][1] + ")");

        // double[][] testCircle = new double[][] {
        //     {-5.0, 0.0},
        //     {0.0, 5.0},
        //     {5.0, 0.0},
        // };
        // System.out.println("Curvature: " + PathGenerator.curvature(testCircle));

        double[][] origin = new double[][] {
            {0.0, 0.0}
        };
        Robot testBot = new Robot(origin, 0.02, 34.0);
        // double[][] unitI = new double[][] {
        //     {1.0, 0.0}
        // };

        // double[][] unitJ = testBot.rotate(unitI, 90.0);
        // System.out.println("This should return the j unit vector: (" + unitJ[0][0] + ", " + unitJ[0][1] + ")");

        // testBot.updatePos(true, 5.0, 990);
        
        // testBot.tankDrive(-10.0, 20.0);

        System.out.println("Number of elements in path: " + testSplinePath.length);

        followPath(testGenerator, testBot);

        System.out.println("Robot position: (" + testBot.robotPos[0][0] + ", " + testBot.robotPos[0][1] + ")");
        System.out.println("Robot angle: " + testBot.robotAngle);
    }

    public static void followPath(PathGenerator generator, Robot robot) {
        double distanceFromTarget = Math.hypot(generator.path[generator.path.length - 1][0] - robot.robotPos[0][0], generator.path[generator.path.length - 1][1] - robot.robotPos[0][1]);
        while(distanceFromTarget > 0.1) {
            generator.updatePos(robot.robotPos, robot.robotAngle);
            robot.tankDrive(generator.velocity(robot.trackLength, true), generator.velocity(robot.trackLength, false));
            distanceFromTarget = Math.hypot(generator.path[generator.path.length - 1][0] - robot.robotPos[0][0], generator.path[generator.path.length - 1][1] - robot.robotPos[0][1]);

            System.out.println("Robot position and angle: (" + robot.robotPos[0][0] + ", " + robot.robotPos[0][1] + ") at " + robot.robotAngle);
            System.out.println("Lookahead point: (" + generator.currentLookaheadPoint[0][0] + ", " + generator.currentLookaheadPoint[0][1] + ")");
            System.out.println("Lookahead curvature: " + generator.lookaheadCurvature());
            System.out.println("Left/Right Velocities: (" + generator.velocity(robot.trackLength, true) + ", " + generator.velocity(robot.trackLength, false) + ")");
        }
    }
}