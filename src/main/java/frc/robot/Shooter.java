package frc.robot;

import frc.robot.PIDMotor;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    public TalonPIDMotor topMotor;
    public TalonPIDMotor bottomMotor;
    public TalonPIDMotor tiltMotor;
    public TalonPIDMotor ballFeedMotor;
    public DigitalInput hopperSwitch;

    public ShootingParameters autoAimCalc;

    final double tiltP = 0.4;
    final double tiltI = 0.0;
    final double tiltD = 0.0;
    final double tiltF = 0.0;
    final double topP = 0.13;
    final double topI = 0.0;
    final double topD = 1.0;
    final double topF = 0.017;
    final double botP = 0.13;
    final double botI = 0.0;
    final double botD = 1.0;
    final double botF = 0.014;
    final double baseLimeAngle = 33.5;
    final double tiltTopSafe = 3.866;

    public double tx, ty, angle;

    double[][] tiltyFixedval, topMotorFixedval, bottomMotorFixedval;

    boolean switchState;
    ZeroedStatus topSwitchZeroed = ZeroedStatus.notZeroed;
    boolean bottomSwitchZeroed = false;

    boolean isManual = false;

    double topMotorSpin;
    double bottomMotorSpin;
    double shooterTilt;

    boolean overrideReverseBallfeed = false;

    double tiltDown = -19400;

    WheelSpinState currentWheelState = WheelSpinState.off;
    AimState currentAimState = AimState.off;
    ShooterState currentShooterState = ShooterState.off;
    NetworkTable limelightVals;

    public Shooter(boolean isCompetitionRobot) {
        topMotor = new TalonPIDMotor(Constants.shootermotor_wheel_top_id, "shooterTopMotor", 4096, topP, topI, topD,
                topF);
        bottomMotor = new TalonPIDMotor(Constants.shootermotor_wheel_bottom_id, "shooterBottomMotor", 4096, botP, botI,
                botD, botF);
        tiltMotor = new TalonPIDMotor(Constants.shootermotor_tilt_id, "shooterTiltMotor", (tiltDown / 50.0), tiltP,
                tiltI, tiltD,
                tiltF);
        // ball feed has no sensors. Thus no PIDF constants.
        ballFeedMotor = new TalonPIDMotor(Constants.ballfeedmotor_id, "ballFeedMotor", 1, 0, 0, 0, 0);
        hopperSwitch = new DigitalInput(Constants.ballfeed_dio_limit_sw);

        topMotor.init();
        bottomMotor.init();
        tiltMotor.init();
        ballFeedMotor.init();

        ballFeedMotor.motor.setNeutralMode(NeutralMode.Brake);

        topMotor.motor.setInverted(true);
        bottomMotor.motor.setSensorPhase(true);
        bottomMotor.motor.setInverted(true);

        tiltMotor.motor.setSelectedSensorPosition(0);
        tiltMotor.motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen);
        tiltMotor.motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen);

        autoAimCalc = new ShootingParameters(isCompetitionRobot);
        if (isCompetitionRobot) {
            // COMPETITION ROBOT
            tiltyFixedval = new double[][] {
                    { 0.0, 36.5 }, // Front
                    { 0.0, 50 }, // Back
                    { 0.0, tiltTopSafe }, // Safe
            };
            topMotorFixedval = new double[][] {
                    { 0.0, 4100 }, // Front
                    { 0.0, 3500 }, // Back
                    { 0.0, 6100 }, // Safe
            };
            bottomMotorFixedval = new double[][] {
                    { 0.0, 4100 }, // Front
                    { 0.0, 4500 }, // Back
                    { 0.0, 5000 }, // Safe
            };
        } else {
            // PRACTICE ROBOT
            tiltyFixedval = new double[][] {
                    { 0.0, 36.5 }, // Front
                    { 0.0, 50 }, // Back
                    { 0.0, tiltTopSafe }, // Safe
            };
            topMotorFixedval = new double[][] {
                    { 0.0, 4100 }, // Front
                    { 0.0, 3500 }, // Back
                    { 0.0, 5500 }, // Safe
            };
            bottomMotorFixedval = new double[][] {
                    { 0.0, 4100 }, // Front
                    { 0.0, 4500 }, // Back
                    { 0.0, 5500 }, // Safe
            };
        }
        limelightVals = NetworkTableInstance.getDefault().getTable("limelight");

    }

    public void printShooterStatus(boolean isEnabled) {
        if (isEnabled) {
            topMotor.printVelocityToLog();
            bottomMotor.printVelocityToLog();
            tiltMotor.printPositionToLog();
        }
        topMotor.printVelocity();

        bottomMotor.printVelocity();

        tiltMotor.printPosition();
    }

    public void spinWheels(WheelSpinState state) {
        currentWheelState = state;
    }

    public void doStuff(double joystickValue) {
      //  SmartDashboard.putBoolean("Hopper Switch", getBallFeedSwitch());
     //   SmartDashboard.putNumber("Top shooter wheel current", topMotor.motor.getMotorOutputPercent());
     //   SmartDashboard.putNumber("Bottom shooter wheel current", bottomMotor.motor.getMotorOutputPercent());
        autoAimCalc.printShooterInterpolation();

        SmartDashboard.putNumber("Tilt Preset Front", tiltyFixedval[0][1]);
        SmartDashboard.putNumber("Tilt Preset Back", tiltyFixedval[1][1]);
        SmartDashboard.putNumber("Tilt Preset Safe", tiltyFixedval[2][1]);
        SmartDashboard.putNumber("Top Preset Front", topMotorFixedval[0][1]);
        SmartDashboard.putNumber("Top Preset Back", topMotorFixedval[1][1]);
        SmartDashboard.putNumber("Top Preset Safe", topMotorFixedval[2][1]);
        SmartDashboard.putNumber("Bottom Preset Front", bottomMotorFixedval[0][1]);
        SmartDashboard.putNumber("Bottom Preset Back", bottomMotorFixedval[1][1]);
        SmartDashboard.putNumber("Bottom Preset Safe", bottomMotorFixedval[2][1]);
        int closestDistanceIndex = autoAimCalc.tilt.getClosestindex(distance());
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putBoolean(Integer.toString(i), false);
        }
        SmartDashboard.putBoolean(Integer.toString(closestDistanceIndex), true);

        // updateTargetAngle();
        distance();

        if (topSwitchZeroed == ZeroedStatus.zeroed) {
            if (currentWheelState == WheelSpinState.on) {
                topMotor.setVelocity(topMotorSpin);
                bottomMotor.setVelocity(bottomMotorSpin);
                setTilt(shooterTilt);
            } else if (currentWheelState == WheelSpinState.off) {
                topMotor.setPercentOutput(0);
                bottomMotor.setPercentOutput(0);
            }
            // TODO the aim stuff

            if (currentShooterState == ShooterState.off) {
                SmartDashboard.putNumber("ShooterState", 0);
                if (overrideReverseBallfeed) {
                    ballFeedMotor.setPercentOutput(-1);
                }
                // else if (getBallFeedSwitch()) {
                // ballFeedMotor.setPercentOutput(0);

                // }
                else {
                    ballFeedMotor.setPercentOutput(0.0);
                }
            } else if (currentShooterState == ShooterState.shootNow) {
               // SmartDashboard.putNumber("ShooterState", 1);
                ballFeedMotor.motor.setVoltage(8);
            } else if (currentShooterState == ShooterState.shootWhenReady) {
               // SmartDashboard.putNumber("ShooterState", 2);
                if (isTargetLocked() && isWheelSpeedLocked()) {
                    ballFeedMotor.motor.setVoltage(8);
                } else {
                    ballFeedMotor.setPercentOutput(0);
                }
            }
            setTilt(joystickValue);
        } else {
            tiltMotor.setPercentOutput(0.2);
            tiltMotor.motor.setSelectedSensorPosition(0);

            // } else if (bottomSwitchZeroed == false) {
            // tiltMotor.setPercentOutput(-0.4);
            // if (tiltMotor.motor.isRevLimitSwitchClosed() == 1) {
            // bottomSwitchZeroed = true;
            // tiltDown = tiltMotor.getPosition();
            // }
            // }
        }
    }

    public void setTilt(double joystickValue) {

        if (isManual) {
            joystickValue = joystickValue * -1;
            if (joystickValue >= 0) {
                double positionValue = (joystickValue * tiltDown);
                tiltMotor.setPosition(positionValue);
                SmartDashboard.putNumber("Shooter Tilt Target", positionValue);
            } else {
                tiltMotor.setPosition(0);
            }
        } else {
            tiltMotor.setPosition(shooterTilt);
        }

    }

    public void shootFrontAtWall() {
        topMotorSpin = topMotorFixedval[0][1];
        bottomMotorSpin = bottomMotorFixedval[0][1];
        shooterTilt = tiltyFixedval[0][1];
    }

    // haha ball go brrrrr lol
    public void shootBackAtWall() {
        topMotorSpin = topMotorFixedval[1][1];
        bottomMotorSpin = bottomMotorFixedval[1][1];
        shooterTilt = tiltyFixedval[1][1];
    }

    public void shootSafeZone() {
        topMotorSpin = topMotorFixedval[2][1];
        bottomMotorSpin = bottomMotorFixedval[2][1];
        shooterTilt = tiltyFixedval[2][1];
    }

    public void stow() {
        topMotorSpin = 0;
        bottomMotorSpin = 0;
        shooterTilt = tiltTopSafe;
    }

    // topMotorSpin = 4500;
    // bottomMotorSpin = 6500;
    // shooterTilt = 3.866;
    // 17.7

    public void autoAim(double distance, double velocity) {

        distance = distance - velocity * (1.0 / 35000.0);

        topMotorSpin = autoAimCalc.speedTop.interpolate(distance);
        bottomMotorSpin = autoAimCalc.speedBot.interpolate(distance);
        if (autoAimCalc.tilt.interpolate(distance) > 0 && autoAimCalc.tilt.interpolate(distance) <= 50) {
            shooterTilt = autoAimCalc.tilt.interpolate(distance);
        } else if (autoAimCalc.tilt.interpolate(distance) < tiltTopSafe) {
            shooterTilt = tiltTopSafe;
        } else if (autoAimCalc.tilt.interpolate(distance) > 50) {
            shooterTilt = 50;
        }
    }

    // Updates and returns angle.
    public double getLimeAngle() {
        angle = baseLimeAngle + tiltMotor.getPosition();
        return angle;
    }

    // Updates the target angles.
    public void updateTargetAngle() {
        // Gets the values from the Limelight and puts it into a table.
        // Gets the horizontal offset between the center crosshair and the object
        // crosshair (tx) in degrees.
        tx = limelightVals.getEntry("tx").getDouble(0);
        SmartDashboard.putNumber("tx", tx);
        // Gets the vertical offset between the center crosshair and the object
        // crosshair (ty) in degrees.
        ty = limelightVals.getEntry("ty").getDouble(0);
        SmartDashboard.putNumber("ty", ty);
    }

    // Gets the distance from the robot to the target.
    public double distance() {
        updateTargetAngle();
        // Equation for calculating distance.
        return (Constants.GoalHeight - Constants.ShooterHeight) / Math.tan(Math.toRadians(this.getLimeAngle() + ty));
    }

    public void aim(AimState state) {
        currentAimState = state;
    }

    public boolean isTargetLocked() {

        if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0) > 0) {

            if (tx <= 8.0 && tx >= -8.0) {
                return true;
            } else {
                return false;
            }

        } else {
            return false;
        }
    }

    public boolean isWheelSpeedLocked() {
        return topMotor.isAtVelocitySetPoint(400) && bottomMotor.isAtVelocitySetPoint(400); // RPM
    }

    public void shoot(ShooterState state) {
        currentShooterState = state;
    }

    public enum WheelSpinState {
        off,
        on,
    }

    public void useCamera(boolean camState) {
        if (camState == true) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        } else {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
        }
    }

    public enum AimState {
        off,
        on,
    }

    public enum ShooterState {
        off,
        shootNow,
        shootWhenReady,
    }

    public enum ZeroedStatus {
        zeroed,
        waiting,
        notZeroed,
    }

    public boolean getBallFeedSwitch() {
        return !hopperSwitch.get();

    }
}
