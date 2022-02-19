package frc.robot.utilityClasses;

import org.ejml.simple.SimpleMatrix;

import frc.robot.Parameters;

import java.util.Arrays;
import java.util.List;

public class CircleFitter {
    private static List<GlobalPoint> circlePoints;
    private static double xsum, ysum, xSquaredsum, ySquaredsum, xysum;
    private static double index1, index2, index3;

    // declare the matrix
    private static SimpleMatrix X = new SimpleMatrix(3, 3);
    private static SimpleMatrix Y = new SimpleMatrix(3, 1);

    private static double A, B, C;
    private static double[] circleData;

    public CircleFitter() {}

    public static void setPoints(List<GlobalPoint> points) {
        circlePoints = points;
        // calculate matrices
        xsum = ysum = xSquaredsum = ySquaredsum = xysum = index1 = index2 = index3 = 0;

        for (int i = 0; i < points.size(); i++) {
            GlobalPoint point = points.get(i);
            double x = point.x;
            double y = point.y;

            xsum += x;
            ysum += y;
            xSquaredsum += x * x;
            ySquaredsum += y * y;
            xysum += x * y;

            index1 += x * (x * x + y * y);
            index2 += y * (x * x + y * y);
            index3 += x * x + y * y;
        }
        // set matrices
        X.set(0, 0, xSquaredsum);
        X.set(0, 1, xysum);
        X.set(0, 2, xsum);

        X.set(1, 0, xysum);
        X.set(1, 1, ySquaredsum);
        X.set(1, 2, ysum);

        X.set(2, 0, xsum);
        X.set(2, 1, ysum);
        X.set(2, 2, circlePoints.size());

        Y.set(0, 0, index1);
        Y.set(1, 0, index2);
        Y.set(2, 0, index3);
    }

    public static double[] calculateCircle() {
        double[] ret = new double[3];

        SimpleMatrix M = X.invert().mult(Y);

        A = M.get(0);
        B = M.get(1);
        C = M.get(2);

        // k value
        ret[0] = A / 2;
        // m value
        ret[1] = B / 2;
        // radius
        ret[2] = Math.sqrt(4 * C + A * A + B * B) / 2;
        // Units are in feet, radius tolerance
        if(Math.abs(ret[2] - 2) > Parameters.shooter.camera.CIRCLE_FIT_TOLERANCE) {
            // set everything to 0 to represent bad data
            Arrays.fill(ret, 0);
        }

        circleData = ret;
        return ret;
    }

    public static double[] getCircleData() {
        return circleData;
    }
}
