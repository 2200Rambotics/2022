package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Climb {
    SparkmaxPIDmotor leftClimb1;
    SparkmaxPIDmotor rightClimb1;
    SparkmaxPIDmotor leftTilt;
    SparkmaxPIDmotor leftClimb2;
    SparkmaxPIDmotor rightClimb2;
    SparkmaxPIDmotor rightTilt;

    edu.wpi.first.wpilibj.Timer climbTimer = new edu.wpi.first.wpilibj.Timer();

    double climbP = 0.2;
    double climbI = 0;
    double climbD = 0;
    double climbF = 0;

    double tiltP = 0.15;
    double tiltI = 0;
    double tiltD = 0;
    double tiltF = 0;

    int climbCurrentLimit = 45;
    double climbMaxAccel = 2; // TODO
    double climbMaxVelocity = 20;

    int tiltCurrentLimit = 45;
    double tiltMaxAccel = 12000; // TODO
    double tiltMaxVelocity = 2000;

    public double climbPositionTop = 87;
    public double climbPositionBottom = 0;
    public double tiltBack = 0;
    public double tiltForward = 36.5;
    public double tiltHook = 33;

    public double climbPositionMiddle = 40;
    public double tiltVertical = 10;

    public int climbStepNumber = -1;

    public Climb() {

        leftClimb1 = new SparkmaxPIDmotor(Constants.climbmotor_left_pull_1_id, "Climb Left 1", 1, climbP, climbI,
                climbD, climbF);
        leftClimb2 = new SparkmaxPIDmotor(Constants.climbmotor_left_pull_2_id, "Climb Left 2", 1, climbP, climbI,
                climbD, climbF);
        leftTilt = new SparkmaxPIDmotor(Constants.climbmotor_left_tilt_id, "Tilt Left", 1, tiltP, tiltI, tiltD, tiltF);
        rightClimb1 = new SparkmaxPIDmotor(Constants.climbmotor_right_pull_1_id, "Climb Right 1", 1, climbP, climbI,
                climbD, climbF);
        rightClimb2 = new SparkmaxPIDmotor(Constants.climbmotor_right_pull_2_id, "Climb Right 2", 1, climbP, climbI,
                climbD, climbF);
        rightTilt = new SparkmaxPIDmotor(Constants.climbmotor_right_tilt_id, "Tilt Right", 1, tiltP, tiltI, tiltD,
                tiltF);

        leftClimb1.init();
        leftClimb2.init();
        leftTilt.init();
        rightClimb1.init();
        rightClimb2.init();
        rightTilt.init();

        leftClimb2.motor.follow(leftClimb1.motor);
        rightClimb2.motor.follow(rightClimb1.motor);

        leftClimb1.motor.getEncoder().setPosition(0);
        leftClimb2.motor.getEncoder().setPosition(0);
        leftTilt.motor.getEncoder().setPosition(0);
        rightClimb1.motor.getEncoder().setPosition(0);
        rightClimb2.motor.getEncoder().setPosition(0);
        rightTilt.motor.getEncoder().setPosition(0);
        for (SparkmaxPIDmotor climbMotor : List.of(leftClimb1, leftClimb2, rightClimb1, rightClimb2)) {
            climbMotor.motor.setSmartCurrentLimit(climbCurrentLimit);
            climbMotor.pid.setSmartMotionMaxAccel(climbMaxAccel, 0);
            climbMotor.pid.setSmartMotionMaxVelocity(climbMaxVelocity, 0);
        }
        for (SparkmaxPIDmotor tiltMotor : List.of(leftTilt, rightTilt)) {
            tiltMotor.motor.setSmartCurrentLimit(tiltCurrentLimit);
            tiltMotor.pid.setSmartMotionMaxAccel(tiltMaxAccel, 0);
            tiltMotor.pid.setSmartMotionMaxVelocity(tiltMaxVelocity, 0);
            // tiltMotor.pid.setOutputRange(-0.3, 0.3);
            // tiltMotor.pid.setSmartMotionAllowedClosedLoopError(allowedErr, slotID)
        }
    }

    public void printClimbStatus(boolean isEnabled) {
        if (isEnabled) {
            leftClimb1.printPositionToLog();
            leftClimb2.printPositionToLog();
            leftTilt.printPositionToLog();
            rightClimb1.printPositionToLog();
            rightClimb2.printPositionToLog();
            rightTilt.printPositionToLog();
            
            // SmartDashboard.putNumber("Left Climb1 StickyFaults", leftClimb1.motor.getStickyFaults());
            // SmartDashboard.putNumber("Left Climb2 StickyFaults", leftClimb2.motor.getStickyFaults());
            // SmartDashboard.putNumber("Left Tilt StickyFaults", leftTilt.motor.getStickyFaults());
            // SmartDashboard.putNumber("Right Climb1 StickyFaults", rightClimb1.motor.getStickyFaults());
            // SmartDashboard.putNumber("Right Climb2 StickyFaults", rightClimb2.motor.getStickyFaults());
            // SmartDashboard.putNumber("Right Tilt StickyFaults", rightTilt.motor.getStickyFaults());
           
        }
        leftClimb1.printPosition();
        leftClimb2.printPosition();
        leftTilt.printPosition();
        rightClimb1.printPosition();
        rightClimb2.printPosition();
        rightTilt.printPosition();
    }

    public void printPIDF() {

        leftClimb1.printPIDF();
        leftClimb2.printPIDF();
        leftTilt.printPIDF();
        rightClimb1.printPIDF();
        rightClimb2.printPIDF();
        rightTilt.printPIDF();

    }

    public void getPIDF() {

        leftClimb1.getPIDF();
        leftClimb2.getPIDF();
        leftTilt.getPIDF();
        rightClimb1.getPIDF();
        rightClimb2.getPIDF();
        rightTilt.getPIDF();

    }

    public void setTilt(double position) {
        // leftTilt.setPosition(position, CANSparkMax.ControlType.kSmartMotion);
        // rightTilt.setPosition(-position, CANSparkMax.ControlType.kSmartMotion);
        leftTilt.setPosition(position);
        rightTilt.setPosition(-position);

    }

    public void setHeight(double height) {
        leftClimb1.setPosition(-height);
        rightClimb1.setPosition(height);
    }

    public void counteract(double tilt) {
        leftTilt.motor.setSmartCurrentLimit(45);
        rightTilt.motor.setSmartCurrentLimit(45);

        double i = (tilt + 20) / 2;
        i = i + 5;

        if (i > 22) {
            i = 22;
        } else if (i < 5) {
            i = 5;
        }
        setTilt(i);

        // if (tilt>8){
        // setTilt(5);

        // }
        // else if( tilt<-8){
        // setTilt(27);
        // }

        // else{
        // setTilt(16);
        // }

    }

    public void tiltOff() {
        leftTilt.setPercentOutput(0);
        rightTilt.setPercentOutput(0);
        leftTilt.motor.setIdleMode(IdleMode.kCoast);
        rightTilt.motor.setIdleMode(IdleMode.kCoast);
    }

    public void tiltBrake() {
        leftTilt.setPercentOutput(0);
        rightTilt.setPercentOutput(0);
        leftTilt.motor.setIdleMode(IdleMode.kBrake);
        rightTilt.motor.setIdleMode(IdleMode.kBrake);
    }

    public void climbOff() {
        leftClimb1.setPercentOutput(0);
        leftClimb2.setPercentOutput(0);
        leftClimb1.motor.setIdleMode(IdleMode.kCoast);
        leftClimb2.motor.setIdleMode(IdleMode.kCoast);
        rightClimb1.setPercentOutput(0);
        rightClimb2.setPercentOutput(0);
        leftClimb1.motor.setIdleMode(IdleMode.kCoast);
        rightClimb2.motor.setIdleMode(IdleMode.kCoast);
    }

    public void climbBrake() {
        leftClimb1.setPercentOutput(0);
        leftClimb2.setPercentOutput(0);
        leftClimb1.motor.setIdleMode(IdleMode.kBrake);
        leftClimb2.motor.setIdleMode(IdleMode.kBrake);
        rightClimb1.setPercentOutput(0);
        rightClimb2.setPercentOutput(0);
        leftClimb1.motor.setIdleMode(IdleMode.kBrake);
        rightClimb2.motor.setIdleMode(IdleMode.kBrake);
    }

    public void climbStage(boolean rightBumper) {

        switch (climbStepNumber) {
            case -1: // home state
                // don't move pls
                this.setTilt(0);
                this.setHeight(0);
                break;

            case 0: // prepare for climb, extend arms
                this.setTilt(this.tiltVertical);
                this.setHeight(this.climbPositionTop);
                break;

            case 1: // start climbing, retracting
                // start autostepping here
                this.setTilt(this.tiltVertical);
                this.setHeight(this.climbPositionMiddle);
                if (leftTilt.isAtPositionSetPoint(10) && rightTilt.isAtPositionSetPoint(10)
                        && leftClimb1.isAtPositionSetPoint(2) && rightClimb1.isAtPositionSetPoint(2)) {
                    climbStepNumber++;
                }
                break;

            case 2: // continue climbing, tilt arms
                this.setTilt(this.tiltForward);
                this.setHeight(this.climbPositionMiddle);
                if (leftTilt.isAtPositionSetPoint(10) && rightTilt.isAtPositionSetPoint(10)
                        && leftClimb1.isAtPositionSetPoint(2) && rightClimb1.isAtPositionSetPoint(2)) {
                    climbStepNumber++;
                }
                break;

            case 3: // continue climbing, retract arms further
                this.setTilt(this.tiltForward);
                this.setHeight(this.climbPositionBottom);
                if (leftTilt.isAtPositionSetPoint(10) && rightTilt.isAtPositionSetPoint(10)
                        && leftClimb1.isAtPositionSetPoint(1) && rightClimb1.isAtPositionSetPoint(1)) {
                    climbTimer.reset();
                    climbTimer.start();
                    climbStepNumber++;
                }
                break;

            case 4: // disable tilt control
                this.tiltOff();
                this.setHeight(this.climbPositionBottom);
                if (climbTimer.get() >= 1 || rightBumper) {
                    climbTimer.stop();
                    climbTimer.reset();
                    climbTimer.start();
                    climbStepNumber++;
                }
                break;

            case 5:
                this.tiltOff();
                this.climbOff();
                if (climbTimer.get() >= 1) {
                    climbTimer.stop();
                    climbTimer.reset();
                    climbTimer.start();
                    climbStepNumber++;

                } else if (rightBumper && climbTimer.get() >= 0.5) {
                    climbTimer.stop();
                    climbTimer.reset();
                    climbTimer.start();
                    climbStepNumber++;
                }
                break;

            case 6:
                this.tiltOff();
                this.setHeight(30);
                if (leftClimb1.isAtPositionSetPoint(2) && rightClimb1.isAtPositionSetPoint(2)) {
                    climbStepNumber++;
                }
                break;

            case 7:
                this.setTilt(this.tiltBack);
                this.setHeight(30);
                if (leftTilt.isAtPositionSetPoint(10) && rightTilt.isAtPositionSetPoint(10)
                        && leftClimb1.isAtPositionSetPoint(2) && rightClimb1.isAtPositionSetPoint(2)) {
                    climbStepNumber++;
                }
                break;

            case 8:
                this.setTilt(tiltBack);
                this.setHeight(climbPositionTop);
                if (leftTilt.isAtPositionSetPoint(10) && rightTilt.isAtPositionSetPoint(10)
                        && leftClimb1.isAtPositionSetPoint(2) && rightClimb1.isAtPositionSetPoint(2)) {
                    climbStepNumber++;
                }
                break;

            case 9:
                this.setTilt(tiltVertical);
                this.setHeight(climbPositionTop);
                break;

            case 10:
                this.setTilt(this.tiltVertical);
                this.setHeight(this.climbPositionMiddle);
                if (leftTilt.isAtPositionSetPoint(10) && rightTilt.isAtPositionSetPoint(10)
                        && leftClimb1.isAtPositionSetPoint(2) && rightClimb1.isAtPositionSetPoint(2)) {
                    climbStepNumber++;
                }
                break;

            case 11:
                this.setTilt(this.tiltForward);
                this.setHeight(this.climbPositionMiddle);
                if (leftTilt.isAtPositionSetPoint(10) && rightTilt.isAtPositionSetPoint(10)
                        && leftClimb1.isAtPositionSetPoint(2) && rightClimb1.isAtPositionSetPoint(2)) {
                    climbStepNumber++;
                }
                break;

            case 12: // continue climbing, retract arms further
                this.setTilt(this.tiltForward);
                this.setHeight(this.climbPositionBottom);
                if (leftTilt.isAtPositionSetPoint(10) && rightTilt.isAtPositionSetPoint(10)
                        && leftClimb1.isAtPositionSetPoint(2) && rightClimb1.isAtPositionSetPoint(2)) {
                    climbTimer.reset();
                    climbTimer.start();
                    climbStepNumber++;
                }
                break;

            case 13: // disable tilt control
                this.tiltOff();
                this.setHeight(this.climbPositionBottom);
                if (climbTimer.get() >= 1 || rightBumper) {
                    climbTimer.stop();
                    climbTimer.reset();
                    climbTimer.start();
                    climbStepNumber++;
                } 
                break;

            case 14:
                // We don't need this step anymore, but we can leave this here
                this.tiltOff();
                this.climbOff();
                if (climbTimer.get() >= 1) {
                    climbTimer.stop();
                    climbTimer.reset();
                    climbTimer.start();
                    climbStepNumber++;

                } else if (rightBumper && climbTimer.get() >= 0.5) {
                    climbTimer.stop();
                    climbTimer.reset();
                    climbTimer.start();
                    climbStepNumber++;
                }
                break;

            case 15:
                this.tiltOff();
                this.setHeight(22);
                if (leftClimb1.isAtPositionSetPoint(2) && rightClimb1.isAtPositionSetPoint(2)) {
                    climbStepNumber++;
                }
                break;

            case 16:
                this.setTilt(this.tiltBack);
                this.setHeight(22);
                if (leftTilt.isAtPositionSetPoint(2) && rightTilt.isAtPositionSetPoint(2)
                        && leftClimb1.isAtPositionSetPoint(2) && rightClimb1.isAtPositionSetPoint(2)) {
                    climbStepNumber++;
                }
                break;

            case 17:
                this.setTilt(tiltBack);
                this.setHeight(climbPositionTop);
                if (leftTilt.isAtPositionSetPoint(10) && rightTilt.isAtPositionSetPoint(10)
                        && leftClimb1.isAtPositionSetPoint(2) && rightClimb1.isAtPositionSetPoint(2)) {
                    climbStepNumber++;
                }
                break;

            case 18:
                this.setTilt(tiltVertical);
                this.setHeight(climbPositionTop);
                break;

            case 19:
                this.setTilt(tiltVertical);
                this.setHeight(climbPositionMiddle);
                if (leftTilt.isAtPositionSetPoint(10) && rightTilt.isAtPositionSetPoint(10)
                        && leftClimb1.isAtPositionSetPoint(2) && rightClimb1.isAtPositionSetPoint(2)) {
                    climbStepNumber++;
                }
                break;

            case 20:
                this.setTilt(tiltVertical);
                this.setHeight(climbPositionMiddle + 20);
                if (leftTilt.isAtPositionSetPoint(10) && rightTilt.isAtPositionSetPoint(10)
                        && leftClimb1.isAtPositionSetPoint(2) && rightClimb1.isAtPositionSetPoint(2)) {
                    climbStepNumber++;
                }
                break;
            case 21:
                this.tiltBrake();
                this.climbBrake();
                break;

        }

    }

    public void goToNextStep() {
        climbStepNumber = climbStepNumber + 1;
    }

    public void goToPreviousStep() {
        climbStepNumber = (climbStepNumber - 1);
        if (climbStepNumber < -1) {
            climbStepNumber = -1;
        }
    }
}
