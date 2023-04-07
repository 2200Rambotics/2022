package frc.robot;

import java.util.spi.ToolProvider;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootingParameters {
    LinearInterpolation tilt;
    LinearInterpolation speedTop;
    LinearInterpolation speedBot;

    public ShootingParameters(boolean isCompetitionRobot) {

        if (isCompetitionRobot) {
            // // COMPETITION ROBOT
            double[][] tiltyval = {
                    { 1.4, 35 },
                    { 6, 23.5, },
                    { 12.5, 4.5 },
                    { 17.1, 3.866 }
            };

            double[][] topspeedyval = {
                    { 1.4, 3800 },
                    { 6, 4100 },
                    { 12.5, 4100 },
                    { 17.1, 4700 }
            };
            double[][] botspeedyval = {
                    { 1.4, 3800 },
                    { 6, 4100 },    
                    { 12.5, 5700 },
                    { 17.1, 6200 }

            };
            tilt = new LinearInterpolation(tiltyval);
            speedTop = new LinearInterpolation(topspeedyval);
            speedBot = new LinearInterpolation(botspeedyval);
        } else {
            // PRACTICE ROBOT
            double[][] tiltyval = {
                    { 1.4, 35 },
                    { 6, 27, },
                    { 12.5, 4.5 },
                    { 17.1, 3.866 }
            };

            double[][] topspeedyval = {
                    { 1.4, 3800 },
                    { 6, 4400 },
                    { 12.5, 4200 },
                    { 17.1, 4900 }
            };
            double[][] botspeedyval = {
                    { 1.4, 3800 },
                    { 6, 4400 },
                    { 12.5, 5800 },
                    { 17.1, 6400 }

            };
            tilt = new LinearInterpolation(tiltyval);
            speedTop = new LinearInterpolation(topspeedyval);
            speedBot = new LinearInterpolation(botspeedyval);
        }

    }

    public void printShooterInterpolation() {
        int i = 0;
        for (double[] d : tilt.values) {
            SmartDashboard.putNumber("Tilt Interpolate " + i, d[1]);
            SmartDashboard.putNumber("Distance Interpolate " + i, d[0]);
            i++;
        }
        i = 0;
        for (double[] d : speedBot.values) {
            SmartDashboard.putNumber("Bottom Motor Interpolate " + i, d[1]);
            i++;
        }
        i = 0;
        for (double[] d : speedTop.values) {
            SmartDashboard.putNumber("Top Motor Interpolate " + i, d[1]);
            i++;
        }

        // SmartDashboard.putNumberArray("Top motor array", topspeedyval);
        // SmartDashboard.putNumberArray("Bottom motor array", botspeedyval);
        // SmartDashboard.putNumberArray("Tilt array", tiltyval);
    }

}
