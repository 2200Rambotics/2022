package frc.robot;

public class LinearInterpolation {

    public double[] [] values;

    public static double interpolateOne(double min, double max, double t)
    {
        return min + (max - min) * t;
    }
    
    public LinearInterpolation(double[][] values){
        this.values = values;
    }

    public double interpolate(double x) {
        double y = 0;
        // y = y1 + ((x-x1)/(x2-x1))*(y2-y1)
        double lastx = values[values.length-1][0];
        for(int i = 0; i<values.length-1; i++){
            double x1 = values[i][0];
            double x2 = values[i+1][0];
            double y1 = values[i][1];
            double y2 = values[i+1][1];

            if (x>=x1 && x<=lastx){
                if (x<=x2) {
                    y = y1 + ((x-x1)/(x2-x1))*(y2-y1); 
                    return y;
                }
            }else if(x<x1){
                y = y1 + ((x-x1)/(x2-x1))*(y2-y1);
                return y;
            }else{
                x1 = values[values.length-2][0];
                x2 = values[values.length-1][0];
                y1 = values[values.length-2][1];
                y2 = values[values.length-1][1];
                y = y1 + ((x-x1)/(x2-x1))*(y2-y1);
                return y;
            }
        }
        return(y);
    }

    public int getClosestindex(double x) {
        double lowestError = Double.POSITIVE_INFINITY;
        int lowestErrorIndex = 0;

        for (int i = 0; i < values.length; i++) {
            if (Math.abs(x - values[i][0]) < lowestError)
            {
                lowestError = Math.abs(x - values[i][0]);
                lowestErrorIndex = i;
            }
        }
        return lowestErrorIndex;
    }
}

