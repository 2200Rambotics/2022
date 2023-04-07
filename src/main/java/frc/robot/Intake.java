package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    WPI_TalonSRX rollerMotor;
    DoubleSolenoid pneuPiston;
    DoubleSolenoid pneuPiston2;
    boolean intakeIsUp;

    public Intake() {
        rollerMotor = new WPI_TalonSRX(Constants.intakemotor_roller_id);
        // pneuPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        // Constants.intake_1_solenoid, Constants.intake_2_solenoid);
        pneuPiston = new DoubleSolenoid(Constants.pcm_id, PneumaticsModuleType.REVPH, Constants.intake_1_solenoid,
                Constants.intake_2_solenoid);
        pneuPiston2 = new DoubleSolenoid(Constants.pcm_id, PneumaticsModuleType.REVPH, Constants.intake_3_solenoid, Constants.intake_4_solenoid); 
        intakeIsUp = true;

    }

    public void intakeUp() {
        pneuPiston.set(Value.kForward);
        pneuPiston2.set(Value.kForward);

    }

    public void intakeNearUp() {
        pneuPiston.set(Value.kForward);
        pneuPiston2.set(Value.kReverse);

    }

    public void intakeNearDown() {
        pneuPiston.set(Value.kReverse);
        pneuPiston2.set(Value.kForward);

    }

    public void intakeDown() {
        pneuPiston.set(Value.kReverse);
        pneuPiston2.set(Value.kReverse);

    }


    public void roller(double speed) {
        rollerMotor.set(speed);
        //printIntakeCurrent();

    }

    public void toggle() {
        if (intakeIsUp) {
            intakeIsUp = false;
            intakeDown();

        } else {
            intakeIsUp = true;
            intakeUp();
        }
    }

    public void printIntakeCurrent() {
        SmartDashboard.putNumber("intake current", rollerMotor.getSupplyCurrent());
    }

    public void setRollerCurrent() {
        rollerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 10, 1));
    }

   
}