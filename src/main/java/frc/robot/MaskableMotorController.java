package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

class MaskableMotorController implements SpeedController {

    boolean mask = false;
    SpeedController og;

    public MaskableMotorController(MotorController m) {
        og = m;
    }

    public void setMask(boolean mask) {
        this.mask = mask;
    }

    @Override
    public void set(double speed) {
        if (!mask) {
            og.set(speed);
        }
    }

    @Override
    public double get() {
        return og.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        if (!mask) {
            og.setInverted(isInverted);
        }
    }

    @Override
    public boolean getInverted() {
        return og.getInverted();
    }

    @Override
    public void disable() {
        og.disable();
    }

    @Override
    public void stopMotor() {
        og.stopMotor();
    }

}