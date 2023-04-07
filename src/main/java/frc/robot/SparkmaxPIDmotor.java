package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkmaxPIDmotor extends PIDMotor{

    public CANSparkMax motor;
    public SparkMaxPIDController pid;

    public SparkmaxPIDmotor(int port, String name, double encoderCountsPerUnit, double pV, double iV, double dV, double fV) {
        super(port, name, encoderCountsPerUnit, pV, iV, dV, fV);
        motor = new CANSparkMax(port, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        pid = motor.getPIDController();
    }

    @Override
    protected void initImpl() {
        // TODO Auto-generated method stub
        setPIDF(p, i, d, f);
    }

    @Override
    protected void setPIDFImpl() {
        // TODO Auto-generated method stub
        pid.setP(p);
        pid.setI(i);
        pid.setD(d);
        pid.setIZone(0);
        pid.setFF(f);
        pid.setOutputRange(-1, 1);
    }

    @Override
    protected void setPositionImpl(double positionSetPoint) {
        // TODO Auto-generated method stub
        pid.setReference(positionSetPoint, CANSparkMax.ControlType.kPosition);
    }

    @Override
    protected void setPositionImpl(double positionSetPoint, CANSparkMax.ControlType controlType) {
        // TODO Auto-generated method stub
        pid.setReference(positionSetPoint, controlType);
    }

    @Override
    protected void setVelocityImpl(double velocitySetPoint) {
        // TODO Auto-generated method stub
        pid.setReference(velocitySetPoint, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    protected double getPositionImpl() {
        // TODO Auto-generated method stub
       return motor.getEncoder().getPosition(); 
    
    }

    @Override
    protected double getVelocityImpl() {
        // TODO Auto-generated method stub
        
        return motor.getEncoder().getVelocity();
    }

    @Override
    protected void setPercentOutputImpl(double percentOutput) {
        // TODO Auto-generated method stub
        motor.set(percentOutput);
    }
}
