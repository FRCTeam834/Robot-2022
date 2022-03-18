import java.util.ArrayList;

public class MovingAverage {

    // The number of data points to store at once
    int dataPtCount;

    // The sum of the points (used so we don't need to recompute sum every time)
    double sum = 0;

    // List of data points
    ArrayList<Double> dataPts = new ArrayList<Double>();

    /**
    * @param dataPtCount The number of data points to consider
    */
    MovingAverage(int dataPtCount) {
        this.dataPtCount = dataPtCount;
    }

    public double addPt(double value) {

        // Add the value to the list of data points
        dataPts.add(value);

        // Add to the sum
        sum += value;

        // Remove the last value if we have too many data points
        if (dataPts.size() > dataPtCount) {
            sum -= dataPts.remove(0);
        }
        
        // Return the average
        return getAverage();
    }

    public double getAverage() {

        // Find the size
        int size = dataPts.size();

        if (size > 0) {
            return (sum / size);
        }
        else {
            return 0;
        }
    }

    public void clearPts() {
        dataPts.clear();
        sum = 0;
    }
    
}