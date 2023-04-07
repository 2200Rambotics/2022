package frc.robot;

import java.util.ArrayList;

import javax.swing.text.SimpleAttributeSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive {
    int driveType;
    public SparkmaxPIDmotor frontRightMotor;
    public SparkmaxPIDmotor frontLeftMotor;
    public SparkmaxPIDmotor backRightMotor;
    public SparkmaxPIDmotor backLeftMotor;

    public MaskableMotorController maskedRightMotor;
    public MaskableMotorController maskedLeftMotor;

    public double rightEncoderLockTarget = 0;
    public double leftEncoderLockTarget = 0;

    public Gyro gyro;

    public Pose2d robotPosition;

    public DoubleSolenoid shiftPist;

    DifferentialDrive drive;

    final double driveCountsPerFoot = -11.5;

    public final double lockP = 0.1;
    public final double lockI = 0;
    public final double lockD = 0;
    public final double lockF = 0;

    public double oldDistance = 0;
    public double oldAngle = 0;
    public double distance = 0;
    public double startRotation = 0;

    public double x = 0;
    public DoubleLogEntry distanceLogLeft;
    public DoubleLogEntry distanceLogRight;

    public Drive(Gyro g) {
        robotPosition = new Pose2d();
        distanceLogLeft = new DoubleLogEntry(DataLogManager.getLog(), "Left drive distance");
        distanceLogRight = new DoubleLogEntry(DataLogManager.getLog(), "Right drive distance");
        gyro = g;
        this.frontRightMotor = new SparkmaxPIDmotor(Constants.drivemotor_right1_id, "drive front right",
                driveCountsPerFoot, .5, 0, 0,
                0);
        this.frontLeftMotor = new SparkmaxPIDmotor(Constants.drivemotor_left1_id, "drive front left",
                driveCountsPerFoot, .5, 0, 0, 0);
        this.backRightMotor = new SparkmaxPIDmotor(Constants.drivemotor_right2_id, "drive front right",
                driveCountsPerFoot, .5, 0, 0,
                0);
        this.backLeftMotor = new SparkmaxPIDmotor(Constants.drivemotor_left2_id, "drive front left", driveCountsPerFoot,
                .5, 0, 0, 0);

        this.backLeftMotor.motor.follow(this.frontLeftMotor.motor);
        this.backRightMotor.motor.follow(this.frontRightMotor.motor);
        this.frontLeftMotor.motor.setInverted(true);

        maskedLeftMotor = new MaskableMotorController(this.frontLeftMotor.motor);
        maskedRightMotor = new MaskableMotorController(this.frontRightMotor.motor);

        maskedLeftMotor.setMask(false);
        maskedRightMotor.setMask(false);

        drive = new DifferentialDrive(maskedLeftMotor, maskedRightMotor);
        this.shiftPist = new DoubleSolenoid(Constants.pcm_id, PneumaticsModuleType.REVPH,
                Constants.driveshifter_1_solenoid, Constants.driveshifter_2_solenoid);
        drive.setDeadband(0.05);
        
    }

    public double getCompass() {
        return gyro.compass() - startRotation;
    }

    public void drivePeriodic() {
        oldDistance = distance;
        distance = getAverageDistance();
        updatePosition(distance - oldDistance, (oldAngle + getCompass()) / 2.0);
        oldAngle = getCompass();
    }

    public void updatePosition(double deltaDistance, double angle) {
        double dx = deltaDistance * Math.sin(Math.toRadians(angle));
        double dy = deltaDistance * Math.cos(Math.toRadians(angle));
        double x = dx + robotPosition.getX();
        double y = dy + robotPosition.getY();
        robotPosition = new Pose2d(new Translation2d(x, y), new Rotation2d(angle));
    }

    public static double clamp(double value, double max, double min) {
        if (value < min) {
            value = min;
        } else if (value > max) {
            value = max;
        }
        return value;
    }

    public void curveDrive(double forward, double turn) {
        maskedLeftMotor.setMask(false);
        maskedRightMotor.setMask(false);
        drive.setSafetyEnabled(true);

        boolean quickTurn = false;
        if (forward > 0.25 || forward < -0.25) {
            quickTurn = false;
        } else {
            quickTurn = true;
        }
        // if (forward < 0.05 && forward > -0.05) {
        // drive.curvatureDrive(0, 0, false);
        // }
        // else {
        drive.curvatureDrive(forward, turn, quickTurn);
        // }
    }

    public void gearShift(boolean highshift) {
        if (highshift == true) {

            shiftPist.set(Value.kForward);

        } else {
            shiftPist.set(Value.kReverse);
        }

    }

    public void lockDrive(boolean updateValues) {
        maskedLeftMotor.setMask(true);
        maskedRightMotor.setMask(true);

        if (updateValues) {
            rightEncoderLockTarget = frontRightMotor.getPosition();
            leftEncoderLockTarget = frontLeftMotor.getPosition();
            frontRightMotor.setPIDF(lockP, lockI, lockD, lockF);
            frontLeftMotor.setPIDF(lockP, lockI, lockD, lockF);
        }
        drive.setSafetyEnabled(false);

        frontRightMotor.setPosition(rightEncoderLockTarget);
        frontLeftMotor.setPosition(leftEncoderLockTarget);
    }

    public boolean driveStraight(double speed, double distance, double marginOfError, double compassAngle) {
        maskedLeftMotor.setMask(false);
        maskedRightMotor.setMask(false);
        double leftDistance = frontLeftMotor.getPosition();
        double rightDistance = frontRightMotor.getPosition();
        double distanceCovered = (leftDistance + rightDistance) / 2.0;
        double slowdrive = -0.5;

        if (speed < 0) {
            slowdrive = -slowdrive;
        }

        double compassError = (compassAngle - gyro.compass()) * 0.05;

        if (Math.abs(distanceCovered) < Math.abs(distance) - 0.5) {
            drive.arcadeDrive(-speed, -compassError);
        } else if (Math.abs(distanceCovered) > Math.abs(distance) - 0.5
                && Math.abs(distanceCovered) < Math.abs(distance)) {
            drive.arcadeDrive(slowdrive, -compassError);
        } else {
            drive.arcadeDrive(0, 0);
        }
        return isAtDistanceSetPoint(distance, marginOfError);
    }

    public void adjustAuto(double tx, double p, double joyVal) {
        maskedLeftMotor.setMask(false);
        maskedRightMotor.setMask(false);
        curveDrive(joyVal, -(p * tx) / 22);
    }

    public boolean autoturn(double maxTurn, double angle, double startAngle, double marginOfError) {
        maskedLeftMotor.setMask(false);
        maskedRightMotor.setMask(false);

        double targetAngle = angle + startAngle;
        double currentLocation = gyro.compass();
        double error = targetAngle - currentLocation;
        SmartDashboard.putNumber("turnError", error);

        double speed = error * 1 / 35.0;
        speed = clamp(speed, maxTurn, -maxTurn);
        drive.arcadeDrive(0, -speed);
        return isAtAngleSetPoint(targetAngle, marginOfError, startAngle);
    }

    public void printDriveDistance() {
        double leftDistance = frontLeftMotor.getPosition();
        double rightDistance = frontRightMotor.getPosition();
        SmartDashboard.putNumber("left drive distance", leftDistance);
        SmartDashboard.putNumber("right drive distance", rightDistance);
    }

    public void printDriveDistanceToLog() {
        double leftDistance = frontLeftMotor.getPosition();
        double rightDistance = frontRightMotor.getPosition();
        distanceLogLeft.append(leftDistance);
        distanceLogRight.append(rightDistance);
    }

    public void zeroEncoders() {
        frontLeftMotor.motor.getEncoder().setPosition(0);
        frontRightMotor.motor.getEncoder().setPosition(0);
        backLeftMotor.motor.getEncoder().setPosition(0);
        backRightMotor.motor.getEncoder().setPosition(0);
    }

    public boolean isAtDistanceSetPoint(double targetDistance, double marginOfError) {
        double leftDistance = frontLeftMotor.getPosition();
        double rightDistance = frontRightMotor.getPosition();
        double distanceCovered = (leftDistance + rightDistance) / 2.0;

        if (distanceCovered > targetDistance - marginOfError && distanceCovered < targetDistance + marginOfError) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isAtAngleSetPoint(double targetAngle, double marginOfError, double startingPoint) {
        double currentLocation = gyro.compass();
        double error = targetAngle - currentLocation;

        double leftMotorVelocity = frontLeftMotor.getVelocity();
        double rightMotorVelocity = frontRightMotor.getVelocity();
        double velocityMarginOfError = 1; // TODO

        if (Math.abs(error) < marginOfError) {
            return true;
            // if(Math.abs(leftMotorVelocity) < velocityMarginOfError &&
            // Math.abs(rightMotorVelocity) < velocityMarginOfError){
            // return true;
            // }else{
            // return false;
            // }
        } else {
            return false;
        }
    }

    public void resetRotation() {
        startRotation = gyro.compass();
    }

    public double getAverageVelocity() {
        return (frontLeftMotor.getVelocity() + frontRightMotor.getVelocity()) / 2.0;
    }

    public double getAverageDistance() {
        double leftDistance = frontLeftMotor.getPosition();
        double rightDistance = frontRightMotor.getPosition();
        double distanceCovered = (leftDistance + rightDistance) / 2.0;
        return distanceCovered;
    }

    public void printDriveStatus(Boolean isEnabled) {

        // frontRightMotor.printPosition();
        // backRightMotor.printPosition();
        // frontLeftMotor.printPosition();
        // backLeftMotor.printPosition();
        // frontRightMotor.printVelocity();
        // backRightMotor.printVelocity();
        // frontLeftMotor.printVelocity();
        // backLeftMotor.printVelocity();

       // SmartDashboard.putNumber("Robot X", robotPosition.getX());
      //  SmartDashboard.putNumber("Robot Y", robotPosition.getY());
      //  SmartDashboard.putNumber("Robot Angle", getCompass());

        printDriveDistance();

        if (isEnabled) {
            frontRightMotor.printPositionToLog();
            backRightMotor.printPositionToLog();
            frontLeftMotor.printPositionToLog();
            backLeftMotor.printPositionToLog();
            frontRightMotor.printVelocityToLog();
            backRightMotor.printVelocityToLog();
            frontLeftMotor.printVelocityToLog();
            backLeftMotor.printVelocityToLog();

            printDriveDistanceToLog();
        }
    }

    public boolean gridDrive(Pose2d targetPosition, double startAngle, double speed) {
        double targetX = targetPosition.getX();
        double targetY = targetPosition.getY();

        double travelX = targetX - robotPosition.getX();
        double travelY = targetY - robotPosition.getY();

        double travelDistance =  Math.sqrt((travelX*travelX) + (travelY*travelY));   
        // getRadians returns degrees, because our rotation object uses degrees, not radians. 
        // Avoid double degree conversion
        double angleError = Math.toDegrees(Math.atan(travelX/travelY)) - (gyro.compass() - startAngle);  
        
        if( angleError <1 && angleError>-1){
            x=0;
        }
        else if( angleError > x){
            x=x+0.4;
        }
        else if (angleError < x){
            x=x-0.4;
        }

        if (x >12){
            x=12;
        }
        else if (x<-12){
            x=-12;
        }
        if(travelDistance > 1) {
            drive.arcadeDrive(-speed*((360-Math.abs(angleError))/360.0), -x*0.05);
        }
        else if(travelDistance > 0.5) {
            drive.arcadeDrive(-speed*(travelDistance), -x*0.05);
        }
        else {
            drive.arcadeDrive(0, 0);
            return true;
        }
        

        SmartDashboard.putNumber("robot position rad", robotPosition.getRotation().getRadians());
        SmartDashboard.putNumber("Angle Error", angleError);
        return false;

    }
}