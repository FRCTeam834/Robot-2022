package frc.robot.utilityClasses;

import java.util.List;

import org.ejml.data.DMatrix2;
import org.ejml.data.DMatrix3;
import org.ejml.data.DMatrix3x3;
import org.ejml.simple.SimpleMatrix;

import frc.robot.subsystems.Vision;

public class CircleFitter {
    static List<VisionPoint> circlePoints;
    static double radius = 0;
    static double[] computerSums = new double[8];
    static double totalX;
    static double totalY;
    static double totalXSquared;
    static double totalYSquared;
    static double totalXY;

    static double index1;
    static double index2;
    static double index3;

    // declare the matrix
    static SimpleMatrix a = new SimpleMatrix(3,3);
    static SimpleMatrix b = new SimpleMatrix(3,1);
    static SimpleMatrix c = new SimpleMatrix(2,1);

    public void CirlceFitter(List<VisionPoint> points, double radius)
    {
        circlePoints = points;
        this.radius = radius;
    }

    private static void computeSums()
    {
        for(int i = 0; i<circlePoints.size(); i++)
        {
            totalX += circlePoints.get(i).getX();
            totalXSquared += Math.pow(circlePoints.get(i).getY(), 2);
            totalY += circlePoints.get(i).getY();
            totalYSquared += Math.pow(circlePoints.get(i).getY(), 2);
            totalXY += circlePoints.get(i).getX() * circlePoints.get(i).getX();
            index1 += circlePoints.get(i).getX() * (Math.pow(circlePoints.get(i).getX(),2)+ Math.pow(circlePoints.get(i).getY(), 2));
            index2 += circlePoints.get(i).getY() * (Math.pow(circlePoints.get(i).getX(),2)+ Math.pow(circlePoints.get(i).getY(), 2));
            index3 += (Math.pow(circlePoints.get(i).getX(),2)+ Math.pow(circlePoints.get(i).getY(), 2));
        }
    }
    private static double[] computeCenter()
    {
        a.set(0, 0, totalXSquared);
        a.set(0, 1, totalXY);
        a.set(0,2,totalX);
        a.set(1, 0, totalXY);
        a.set(1,1,totalYSquared);
        a.set(1,2,totalY);
        a.set(2, 0, totalX);
        a.set(2, 1, totalY);
        a.set(2, 2, circlePoints.size());
        
        b.set(0, 0, index1);
        b.set(1, 0, index2);
        b.set(2, 0, index3);
        c = a.invert().mult(b);
        return new double[2];
    }
    public static void main(String args[])
    {
        computeSums();
        computeCenter();
    }
    
}
