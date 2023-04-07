package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {

    AHRS ahrs;

    public Gyro(){
        ahrs = new AHRS(SPI.Port.kMXP);
    }

    public void printAllaccel(){
        SmartDashboard.putNumber("Gyro Angle", ahrs.getAngle());
    }

    public double getYAccel(){
        return ahrs.getRawAccelY();
    }
    public double getPitch(){
        return ahrs.getPitch();
    }

    public double compass(){
        return ahrs.getAngle();
    }
}


