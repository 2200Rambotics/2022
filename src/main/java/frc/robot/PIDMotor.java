package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class PIDMotor {
    public double defaultp;
    public double defaulti;
    public double defaultd;
    public double defaultf;
    public double p;
    public double i;
    public double d;
    public double f;
    public String name;
    double encoderCountsPerUnit;
    double positionSetPoint;
    double velocitySetPoint;
    DoubleLogEntry positionLog;
    DoubleLogEntry velocityLog;

    // Calls Motor with extra steps
    public PIDMotor(int port, String name, double encoderCountsPerUnit) {
        this(port, name, encoderCountsPerUnit, 0, 0, 0, 0);
    }

    /*
     * Creates an object that allows for TalonSRX use that makes some parts easier
     */
    public PIDMotor(int port, String name, double encoderCountsPerUnit, double pV, double iV, double dV, double fV) {
        this.name = name;
        p = pV;
        i = iV;
        d = dV;
        f = fV;
        defaultp = p;
        defaulti = i;
        defaultd = d;
        defaultf = f;
        positionSetPoint = 0;
        velocitySetPoint = 0;
        this.encoderCountsPerUnit = encoderCountsPerUnit;
        positionLog = new DoubleLogEntry(DataLogManager.getLog(), name + " position");
        velocityLog = new DoubleLogEntry(DataLogManager.getLog(), name + " velocity");
    }

    // Must be called after constructor
    public void init() {
        setDefaultPIDF();
        initImpl();

    }

    protected abstract void initImpl();

    // Change the encoder counts needed per unit of choice
    public void setEncoderCountsPerUnit(double input) {
        encoderCountsPerUnit = input;
    }

    public void setDefaultPIDF() {
        setPIDF(defaultp, defaulti, defaultd, defaultf);
    }

    // Set PID for the motor to tune the settings
    public void setPIDF(double pV, double iV, double dV, double fV) {
        p = pV;
        i = iV;
        d = dV;
        f = fV;
        setPIDFImpl();
    }

    protected abstract void setPIDFImpl();

    // For position control
    public void setPosition(double position) {
        positionSetPoint = unitsToCounts(position);
        setPositionImpl(positionSetPoint);
    }

    public void setPosition(double position, CANSparkMax.ControlType controlType) {
        positionSetPoint = unitsToCounts(position);
        setPositionImpl(positionSetPoint, controlType);
    }

    public double getPosition() {
        return countsToUnits(getPositionImpl());

    }

    protected abstract void setPositionImpl(double positionSetPoint);

    protected abstract void setPositionImpl(double positionSetPoint, CANSparkMax.ControlType controlType);

    // Move at speed
    public void setVelocity(double velocity) {
        velocitySetPoint = unitsToCounts(velocity) / 600.0;
        setVelocityImpl(velocitySetPoint);
    }

    public double getVelocity() {
        return countsToUnits(getVelocityImpl()) * 600;

    }

    protected abstract void setVelocityImpl(double velocitySetPoint);

    protected abstract double getPositionImpl();

    protected abstract double getVelocityImpl();

    // Check to see if velocity is within a margin of error
    public boolean isAtVelocitySetPoint(double marginOfError) {
        return isNumberinRange(countsToUnits(velocitySetPoint) * 600.0,
                countsToUnits(getVelocityImpl()) * 600.0, marginOfError);
    }

    // Check to see if position is within a margin of error
    public boolean isAtPositionSetPoint(double marginOfError) {
        return isNumberinRange(positionSetPoint, countsToUnits(getPositionImpl()), marginOfError);
    }

    // Check to see if a number is within a range
    private boolean isNumberinRange(double needle, double haystack, double range) {
        return (((haystack - range) < needle) // greater than low bounds
                && (needle < (haystack + range))); // less than upper bounds
    }

    // set a motor to output a certain percent of power
    public void setPercentOutput(double value) {
        setPercentOutputImpl(value);
    }

    protected abstract void setPercentOutputImpl(double percentOutput);

    // Converts input units to encoder counts
    public double unitsToCounts(double units) {
        return units * encoderCountsPerUnit;
    }

    // Turns encoder value into a specific unit
    public double countsToUnits(double counts) {
        return counts / encoderCountsPerUnit;
    }

    // Puts PID values to smartdashboard
    public void printPIDF() {
        SmartDashboard.putNumber("P " + name, p);
        SmartDashboard.putNumber("I " + name, i);
        SmartDashboard.putNumber("D " + name, d);
        SmartDashboard.putNumber("F " + name, f);
    }

    // Grabs PID from smartdashboard
    public void getPIDF() {
        p = SmartDashboard.getNumber("P " + name, p);
        i = SmartDashboard.getNumber("I " + name, i);
        d = SmartDashboard.getNumber("D " + name, d);
        f = SmartDashboard.getNumber("F " + name, f);
        setPIDF(p, i, d, f);
    }

    public void printPosition() {
        printPositionToDashboard();
    }

    public void printPositionToDashboard() {
        double position = getPosition();
        SmartDashboard.putNumber("Position " + name, position);
    }

    public void printPositionToLog() {
        double position = getPosition();
        if (positionLog == null) {
            System.out.println("HELP! logger is null");
            return;
        }
        positionLog.append(position);
        try{
        } catch (Exception e){}
    }

    public void printVelocity() {
        printVelocityToDashboard();
    }

    public void printVelocityToDashboard() {
        SmartDashboard.putNumber("Velocity " + name, getVelocity());
    }

    public void printVelocityToLog() {
        double velocity = getVelocity();
        if (velocityLog == null) {
            System.out.println("HELP! logger is null :(");
            return;
        }
        velocityLog.append(velocity);
        try{
        } catch (IndexOutOfBoundsException e){}
    }
}