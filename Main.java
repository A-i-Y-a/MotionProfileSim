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
        testLineSpline.calculateCoeffs();
        // for(int i = 0; i < 6; ++i) {
        //     System.out.println("Coefficient of x^" + i + ": " + testLineSpline.xCoeffs[i] + "; coefficient of y^" + i + ": " + testLineSpline.yCoeffs[i]);
        // }
        // for(int j = 0; j < 101; ++j) {
        //     double[][] tempPos = testLineSpline.getPosition(j / 100.0);
        //     System.out.println("Position: " + "(" + tempPos[0][0] + ", " + tempPos[0][1] + ")");
        // }
        // System.out.println("Arc length: " + testLineSpline.arcLength(0.000001));

        PathGenerator testGenerator = new PathGenerator(testLineSpline, 10.0, 2.0, 12.0);
        for(int k = 0; k < 24; ++k) {
            // System.out.println("Waypoint " + k + " position: (" + testGenerator.path[k][0] + ", " + testGenerator.path[k][1] + ")");

            // System.out.println("Waypoint " + k + " velocity: " + testGenerator.segV[k]);
        }

        double[][] testPos = {
            {50.0, 50.0},
        };
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
    }
}