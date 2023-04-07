package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Shooter.AimState;
import frc.robot.Shooter.ShooterState;
import frc.robot.Shooter.WheelSpinState;
import frc.robot.Shooter.ZeroedStatus;

public class Autonomous {
  public static int autostep = 0;
  public static double compassStartPoint = 0;

  static fieldSet field;

  public static Timer autoTimer = new Timer();

  public static void fourBallRightSideAuto(Robot r) {
    r.shooter.doStuff(0);
    r.climber.setTilt(0);

    switch (autostep) {
      case 0:
        // reset encoders
        r.drive.zeroEncoders();
        compassStartPoint = r.gyro.compass();
        r.intake.intakeDown();
        r.intake.roller(1);
        autostep++;
        break;
      case 1:
        r.shooter.topSwitchZeroed = ZeroedStatus.notZeroed;
        r.intake.intakeDown();
        r.intake.roller(1);
        if (r.drive.driveStraight(0.8, 2.5, 0.5, compassStartPoint)) {
          autoTimer.reset();
          autoTimer.start();
          r.shooter.topSwitchZeroed = ZeroedStatus.zeroed;
          autostep++;
        }
        break;
      case 2:
        if (autoTimer.get() > 0.5) {
          autostep++;
          r.drive.zeroEncoders();
        }

        break;
      case 3:
        if (r.drive.driveStraight(-0.8, -2.5, 0.5, compassStartPoint)) {
          autostep++;
        }

        break;
      case 4:
        r.intake.intakeUp();
        r.intake.roller(0);
        r.shooter.useCamera(true);
        if (r.drive.autoturn(0.75, 180, compassStartPoint, 10)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();

        }
        break;

      case 5:
        r.intake.roller(1); // Change to -1?
        // r.intake.intakeUp();
        r.shooter.useCamera(true);
        r.drive.adjustAuto(r.shooter.tx, 0.65, 0);
        r.shooter.aim(AimState.on);
        r.shooter.autoAim(r.shooter.distance(), r.drive.getAverageVelocity());
        r.shooter.spinWheels(WheelSpinState.on);
        r.shooter.shoot(ShooterState.shootWhenReady);
        if (autoTimer.get() > 2.0) {
          autostep++;
          r.drive.zeroEncoders();
          r.shooter.stow();
        }
        break;
      case 6:
        r.intake.intakeDown();
        r.intake.roller(1);
        r.shooter.spinWheels(WheelSpinState.off);
        r.shooter.shoot(ShooterState.off);
        r.shooter.useCamera(false);
        if (r.drive.autoturn(0.8, 90, compassStartPoint, 5)) {
          autostep++;
          r.drive.zeroEncoders();
        }
        break;
      case 7:
        r.intake.intakeDown();
        r.intake.roller(1);
        if (r.drive.driveStraight(1.0, 7, 0.5, compassStartPoint + 90)) {
          autostep++;
          r.drive.zeroEncoders();
        }
        break;
      case 8:
        r.intake.intakeDown();
        r.intake.roller(1);
        if (r.drive.driveStraight(1.0, 12, 0.5, compassStartPoint + 82)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();
          r.drive.zeroEncoders();
        }
        break;
      case 9:
        if (autoTimer.get() > 0.5) {
          autostep++;
          r.drive.zeroEncoders();
        }

        break;

      case 10:
        r.intake.roller(0);
        if (r.drive.driveStraight(-1.0, -15, 0.5, compassStartPoint + 82)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();
          r.drive.zeroEncoders();
        }
        break;

      case 11:
        r.intake.intakeUp();
        r.intake.roller(0);
        r.shooter.useCamera(true);
        if (r.drive.autoturn(0.7, 240, compassStartPoint, 10)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();
        }
        break;
      case 12:
        r.intake.roller(1);
        r.shooter.useCamera(true);
        r.drive.adjustAuto(r.shooter.tx, 0.65, 0);
        r.shooter.aim(AimState.on);
        r.shooter.autoAim(r.shooter.distance(), r.drive.getAverageVelocity());
        r.shooter.spinWheels(WheelSpinState.on);
        r.shooter.shoot(ShooterState.shootWhenReady);
        if (autoTimer.get() > 2.5) {
          autostep++;
          r.drive.zeroEncoders();
        }

        break;
    }

  }

  public static void twoBallLeftSideAuto(Robot r) {
    r.shooter.doStuff(0);
    r.climber.setTilt(0);

    switch (autostep) {
      case 0:
        // reset encoders
        r.drive.zeroEncoders();
        compassStartPoint = r.gyro.compass();
        r.intake.intakeDown();
        r.intake.roller(1);
        autostep++;
        break;
      case 1:
        r.shooter.topSwitchZeroed = ZeroedStatus.notZeroed;
        r.intake.intakeDown();
        r.intake.roller(1);
        if (r.drive.driveStraight(0.8, 5.5, 0.5, compassStartPoint)) {
          autoTimer.reset();
          autoTimer.start();
          r.shooter.topSwitchZeroed = ZeroedStatus.zeroed;
          autostep++;
        }
        break;
      case 2:
        if (autoTimer.get() > 0.5) {
          autostep++;
          r.drive.zeroEncoders();
        }

        break;
      case 3:
        if (r.drive.driveStraight(-0.8, -2.5, 0.5, compassStartPoint)) {
          autostep++;
        }

        break;
      case 4:
        r.intake.roller(0);
        r.shooter.useCamera(true);
        if (r.drive.autoturn(0.75, 170, compassStartPoint, 10)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();

        }
        break;

      case 5:
        r.intake.roller(1);
        r.shooter.useCamera(true);
        r.drive.adjustAuto(r.shooter.tx, 0.65, 0);
        r.shooter.aim(AimState.on);
        r.shooter.autoAim(r.shooter.distance(), r.drive.getAverageVelocity());
        r.shooter.spinWheels(WheelSpinState.on);
        r.shooter.shoot(ShooterState.shootWhenReady);
        if (autoTimer.get() > 4.0) {
          autostep++;
          r.drive.zeroEncoders();
          r.shooter.stow();
          r.intake.roller(0);
          r.shooter.shoot(ShooterState.off);
        }
        break;
    }

  }

  public static void threeBallLeftSideAuto(Robot r) {
    r.shooter.doStuff(0);
    r.climber.setTilt(0);

    switch (autostep) {
      case 0:
        // reset encoders
        r.drive.zeroEncoders();
        compassStartPoint = r.gyro.compass();
        r.intake.intakeDown();
        r.intake.roller(1);
        autostep++;
        break;

      case 1:
        r.shooter.topSwitchZeroed = ZeroedStatus.notZeroed;
        r.intake.intakeDown();
        r.intake.roller(1);
        if (r.drive.driveStraight(0.8, 5.5, 0.5, compassStartPoint)) {
          autoTimer.reset();
          autoTimer.start();
          r.shooter.topSwitchZeroed = ZeroedStatus.zeroed;
          autostep++;
        }
        break;

      case 2:
        if (autoTimer.get() > 0.5) {
          autostep++;
          r.drive.zeroEncoders();
        }

        break;

      case 3:
        if (r.drive.driveStraight(-0.8, -2.5, 0.5, compassStartPoint)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();
        }

        break;

      case 4:
        r.intake.roller(0);
        // r.intake.intakeUp();
        // if (autoTimer.get() > 0.2 && autoTimer.get() < 0.3) {
        // r.intake.roller(-1);
        // } else {
        // r.intake.roller(0);
        // }

        r.shooter.useCamera(true);
        if (r.drive.autoturn(0.75, 170, compassStartPoint, 10)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();

        }
        break;

      case 5:
        r.intake.roller(1);
        r.shooter.useCamera(true);
        r.drive.adjustAuto(r.shooter.tx, 0.65, 0);
        r.shooter.aim(AimState.on);
        r.shooter.autoAim(r.shooter.distance(), r.drive.getAverageVelocity());
        r.shooter.spinWheels(WheelSpinState.on);
        r.shooter.shoot(ShooterState.shootWhenReady);
        if (autoTimer.get() > 2.0) {
          autostep++;
          r.drive.zeroEncoders();
          r.shooter.stow();
          r.intake.roller(0);
          r.shooter.shoot(ShooterState.off);
          r.shooter.spinWheels(WheelSpinState.off);
          r.shooter.useCamera(false);
        }
        break;

      case 6:
        r.intake.roller(0);
        if (r.drive.autoturn(0.75, 265, compassStartPoint, 5)) {
          autostep++;
          r.drive.zeroEncoders();
        }
        break;

      case 7:
        if (r.drive.driveStraight(1.0, 16.7, 0.5, compassStartPoint + 265 + 7.27)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();
          r.drive.zeroEncoders();
        }
        if (r.drive.getAverageDistance() >= 10) {
          r.intake.intakeNearDown();
          r.intake.roller(1);

        }
        break;

      case 8:
        r.intake.roller(1);
        if (autoTimer.get() > 0.5) {
          r.intake.intakeDown();
        } else {
          r.intake.intakeNearDown();
        }
        if (autoTimer.get() > 0.75) {
          autostep++;
          r.drive.zeroEncoders();
        }
        break;

      case 9:
        r.intake.intakeNearDown();
        r.intake.roller(0);
        if (r.drive.driveStraight(-1.0, -1, 0.5, compassStartPoint + 265)) {
          autoTimer.reset();
          autoTimer.start();
          r.intake.intakeNearDown();
          autostep++;
        }
        break;

      case 10:
        r.intake.intakeNearDown();
        r.intake.roller(1);

        if (autoTimer.get() > 0.95) {
          autostep++;
          r.drive.zeroEncoders();
          autoTimer.reset();
          autoTimer.start();
        }
        break;

      case 11:
        r.intake.intakeDown();
        r.intake.roller(1);

        if (autoTimer.get() > 0.25) {
          autostep++;
          r.drive.zeroEncoders();
          autoTimer.reset();
          autoTimer.start();
        }
        break;

      case 12:
        r.intake.intakeUp();
        r.intake.roller(0);
        if (r.drive.driveStraight(-1.0, -9, 0.5, compassStartPoint + 265)) {
          autostep++;
        }
        break;

      case 13:
        if (r.drive.autoturn(0.75, 150, compassStartPoint, 5)) {
          autostep++;
          autoTimer.stop();
          autoTimer.reset();
          autoTimer.start();
        }
        break;

      case 14:
        r.intake.roller(1);
        r.shooter.useCamera(true);
        r.drive.adjustAuto(r.shooter.tx, 0.65, 0);
        r.shooter.aim(AimState.on);
        r.shooter.autoAim(r.shooter.distance(), r.drive.getAverageVelocity());
        r.shooter.spinWheels(WheelSpinState.on);
        r.shooter.shoot(ShooterState.shootWhenReady);
        if (autoTimer.get() > 2.0) {
          // autostep++;
          r.drive.zeroEncoders();
        }
        break;

    }
  }

  public static void stealTwoBallLeftSideAuto(Robot r) {
    r.shooter.doStuff(0);
    r.climber.setTilt(0);

    switch (autostep) {
      case 0:
        // reset encoders
        r.drive.zeroEncoders();
        compassStartPoint = r.gyro.compass();
        r.intake.intakeDown();
        r.intake.roller(1);
        autostep++;
        break;
      case 1:
        r.shooter.topSwitchZeroed = ZeroedStatus.notZeroed;
        r.intake.intakeDown();
        r.intake.roller(1);
        if (r.drive.driveStraight(0.8, 5.5, 0.5, compassStartPoint)) {
          autoTimer.reset();
          autoTimer.start();
          r.shooter.topSwitchZeroed = ZeroedStatus.zeroed;
          autostep++;
        }
        break;
      case 2:
        if (autoTimer.get() > 0.5) {
          autostep++;
          r.drive.zeroEncoders();
        }

        break;
      case 3:
        if (r.drive.driveStraight(-0.8, -2, 0.5, compassStartPoint)) {
          autostep++;
        }

        break;
      case 4:
        r.intake.roller(0);
        r.shooter.useCamera(true);
        if (r.drive.autoturn(0.8, 170, compassStartPoint, 10)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();

        }
        break;

      case 5:
        r.intake.roller(1);
        r.shooter.useCamera(true);
        r.drive.adjustAuto(r.shooter.tx, 0.65, 0);
        r.shooter.aim(AimState.on);
        r.shooter.autoAim(r.shooter.distance(), r.drive.getAverageVelocity());
        r.shooter.spinWheels(WheelSpinState.on);
        r.shooter.shoot(ShooterState.shootWhenReady);
        if (autoTimer.get() > 1.95) {
          autostep++;
          r.drive.zeroEncoders();
          r.shooter.stow();
          r.intake.roller(0);
          r.shooter.shoot(ShooterState.off);
        }
        break;

      case 6:
        if (r.drive.autoturn(0.75, 90, compassStartPoint, 10)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();
          r.drive.zeroEncoders();
        }
        break;

      case 7:
        r.intake.intakeNearDown();
        r.intake.roller(1);
        if (r.drive.driveStraight(0.8, 4.5, 0.5, compassStartPoint + 75)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();
        }
        break;

      case 8:
        r.intake.roller(1);
        r.intake.intakeDown();
        if (autoTimer.get() > 0.6) {
          r.intake.roller(0);
          autostep++;
          r.drive.zeroEncoders();
        }
        break;

      case 9:
        if (r.drive.autoturn(0.75, 220, compassStartPoint, 10)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();
          r.drive.zeroEncoders();
        }
        break;

      case 10:
        r.intake.intakeNearDown();
        r.intake.roller(1);
        if (r.drive.driveStraight(1, 12, 0.5, compassStartPoint + 240)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();
        }

        break;

      case 11:
        r.intake.intakeDown();
        r.intake.roller(1);
        if (autoTimer.get() > 0.5) {
          r.intake.roller(0);
          autostep++;
          r.drive.zeroEncoders();
        }
        break;

      case 12:
        r.intake.intakeNearDown();
        r.intake.roller(0);
        if (r.drive.driveStraight(-1, -5, 0.5, compassStartPoint + 250)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();
        }

        break;

      case 13:
        r.intake.roller(0);
        r.intake.intakeUp();
        if (r.drive.autoturn(0.75, 165, compassStartPoint, 10)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();
          r.drive.zeroEncoders();

        }
        break;

      case 14:
        if (r.drive.driveStraight(1, 5, 0.5, compassStartPoint + 155)) {
          autostep++;
          autoTimer.reset();
          autoTimer.start();
        }
        break;

      case 15:
        r.intake.roller(-0.65);
        r.shooter.ballFeedMotor.setPercentOutput(-1);
        break;
    }

  }

  public static Pose2d creatPose(double x, double y, double rotation) {
    return new Pose2d(x, y, new Rotation2d(rotation));
  }


  public enum fieldSet{
    sciBlue,
    sciRed,
    techBlue,
    techRed,
  }
}