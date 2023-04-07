package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

public class TalonPIDMotor extends PIDMotor {
    public WPI_TalonSRX motor;

    public TalonPIDMotor(int port, String name, double encoderCountsPerUnit, double pV, double iV, double dV,
            double fV) {
        super(port, name, encoderCountsPerUnit, pV, iV, dV, fV);
        motor = new WPI_TalonSRX(port);
    }

    @Override
    protected void initImpl() {
        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    }

    @Override
    protected void setPIDFImpl() {
        motor.config_kP(0, p);
        motor.config_kI(0, i);
        motor.config_kD(0, d);
        motor.config_kF(0, f);
    }

    @Override
    protected void setPositionImpl(double positionSetPoint) {
        motor.set(ControlMode.Position, positionSetPoint);
    }

    @Override
    protected void setPositionImpl(double positionSetPoint, CANSparkMax.ControlType controlType) {
        // This method is a hack. It does not support control mode
        motor.set(ControlMode.Position, positionSetPoint);
    }

    @Override
    protected void setVelocityImpl(double velocitySetPoint) {
        motor.set(ControlMode.Velocity, velocitySetPoint);
    }

    @Override
    protected double getPositionImpl() {
        return motor.getSelectedSensorPosition();
    }

    @Override
    protected double getVelocityImpl() {
        return motor.getSelectedSensorVelocity();
    }

    @Override
    protected void setPercentOutputImpl(double percentOutput) {
        motor.set(ControlMode.PercentOutput, percentOutput);
    }
}
