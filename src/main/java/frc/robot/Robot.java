// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Timer;

import com.revrobotics.CANSparkMax.IdleMode;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Shooter.AimState;
import frc.robot.Shooter.ShooterState;
import frc.robot.Shooter.WheelSpinState;
import frc.robot.Shooter.ZeroedStatus;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String twoBall = "Two ball left side";
  private static final String threeBall = "Three ball left side";
  private static final String fourBall = "Four ball right side";
  private static final String stealTwoBallLeftSide = "steal two ball left side";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static final String fieldOneBlue = "Field Sci Blue";
  private static final String fieldOneRed = "Field Sci Red";
  private static final String fieldTwoBlue = "Field Tech Blue";
  private static final String fieldTwoRed = "Field Tech Red";
  private String m_fieldSelected;
  private final SendableChooser<String> m_fieldChooser = new SendableChooser<>();

  private XboxController joystick = new XboxController(0); // Driver
  private XboxController coJoystick = new XboxController(1); // Co-Driver
  private XboxController calibrationJoystick = new XboxController(2); // Calibrator/Human Player

  public Shooter shooter;

  public boolean camState = false;
  public boolean isAdjusting = false;

  public Intake intake;

  public Gyro gyro;

  public PneumaticHub pneu;
  public Drive drive;

  public Climb climber;

  public edu.wpi.first.wpilibj.Timer pisTime = new edu.wpi.first.wpilibj.Timer();

  double climbTiltPosition = 0;
  double climbHeightPosition = 0;

  private double distance = 0;
  public double ballX = 0;
  public double ballY = 0;

  public boolean calibrationA;
  public boolean calibrationB;
  public boolean calibrationX;
  public boolean calibrationY;
  public boolean calibrationLBumper;
  public boolean calibrationRBumper;
  public boolean calibrationBackButton;
  public boolean calibrationStartButton;
  public int calibrateInterpolationIndex = 0;
  public boolean debounceCalibration = false;
  public boolean updateValues = false;

  public DataLogManager log;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(true);

    // ADAM CHANGE THIS IF YOU ARE UPLOADING TO THE COMPETITION OR PRACTICE ROBOT
    // true -> Competition robot
    // false -> Practice robot
    shooter = new Shooter(true);
    gyro = new Gyro();
    intake = new Intake();
    climber = new Climb();
    pneu = new PneumaticHub(Constants.pcm_id);
    drive = new Drive(gyro);

    drive.resetRotation();

    intake.intakeUp();
    intake.setRollerCurrent();
    m_chooser.addOption("Four ball auto from right side", fourBall);
    m_chooser.addOption("Three ball auto from left side", threeBall);
    m_chooser.setDefaultOption("Two ball auto from left side", twoBall);
    m_chooser.addOption("steal Two ball auto from left side", stealTwoBallLeftSide);

    SmartDashboard.putData("Auto choices7", m_chooser);

    m_fieldChooser.setDefaultOption("Field Sci Blue", fieldOneBlue);
    m_fieldChooser.addOption("Field Sci Red", fieldOneRed);
    m_fieldChooser.addOption("Field Tech Blue", fieldTwoBlue);
    m_fieldChooser.addOption("Field Tech Red", fieldTwoRed);

    SmartDashboard.putData("Field Choices2", m_fieldChooser);

    // climber.leftClimb1.motor.setIdleMode(IdleMode.kBrake);
    // climber.leftClimb2.motor.setIdleMode(IdleMode.kBrake);
    // climber.rightClimb1.motor.setIdleMode(IdleMode.kBrake);
    // climber.rightClimb2.motor.setIdleMode(IdleMode.kBrake);
    // shooter.topMotor.printPIDF();
    // shooter.bottomMotor.printPIDF();
    // shooter.tiltMotor.printPIDF();

    // drive.frontRightMotor.printPIDF();
    // drive.backRightMotor.printPIDF();
    // drive.frontLeftMotor.printPIDF();
    // drive.backLeftMotor.printPIDF();

    drive.zeroEncoders();

    // climber.printPIDF();

    pneu.enableCompressorAnalog(100, 110);

    SmartDashboard.putNumber("climbTiltPosition", climbTiltPosition);
    SmartDashboard.putNumber("climbHeightPosition", climbHeightPosition);
    SmartDashboard.putNumber("tx", 0);
    SmartDashboard.putNumber("ty", 0);
    SmartDashboard.putNumber("distance", distance);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("Compass", gyro.compass());

    if (pneu.getPressure(0) > 60) {
      SmartDashboard.putBoolean("Pressure", true);
    } else {
      SmartDashboard.putBoolean("Pressure", false);
    }
    SmartDashboard.putNumber("Pressure PSI", pneu.getPressure(0));

    drive.printDriveDistance();
    //drive.drivePeriodic();

    climber.printClimbStatus(isEnabled());
    drive.printDriveStatus(isEnabled());
    shooter.printShooterStatus(isEnabled());

    distance = shooter.distance();
    SmartDashboard.putNumber("distance", distance);

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    Autonomous.autostep = 0;
    SmartDashboard.putString("m_autoselected", m_autoSelected);
    
    switch (m_fieldChooser.getSelected()) {
      case fieldOneBlue:
        Autonomous.field = Autonomous.fieldSet.sciBlue;
        break;
      case fieldOneRed:
        Autonomous.field = Autonomous.fieldSet.sciRed;
        break;
      case fieldTwoBlue:
        Autonomous.field = Autonomous.fieldSet.techBlue;
        break;
      case fieldTwoRed:
        Autonomous.field = Autonomous.fieldSet.techRed;
        break;

    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("AutoStep", Autonomous.autostep);
    SmartDashboard.putString("m_autoselected", m_autoSelected);
    // m_autoSelected = twoBall;
    switch (m_autoSelected) {
      case fourBall:
        Autonomous.fourBallRightSideAuto(this);
        break;
      case threeBall:
        Autonomous.threeBallLeftSideAuto(this);
        break;
      case twoBall:
        Autonomous.twoBallLeftSideAuto(this);
        break;
      case stealTwoBallLeftSide:
        Autonomous.stealTwoBallLeftSideAuto(this);
        break;
      default:
        // Put default auto code here
        break;
    }

    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double dLeftX = joystick.getLeftX();
    double dLeftY = joystick.getLeftY();
    

    double cLeftX = coJoystick.getLeftX();
    double cLeftY = coJoystick.getLeftY();
    
    gyro.printAllaccel();

    // Drive Controls
    if (joystick.getRightTriggerAxis() > 0.8) {
      drive.lockDrive(updateValues);
      updateValues = false;
    } else {
      updateValues = true;
      if (isAdjusting) {
        shooter.useCamera(true);
        drive.adjustAuto(shooter.tx + (dLeftX * 20), 0.65, dLeftY);
      } else {
        // cam2.setDriverMode(true);
        if (cLeftY <= 0.08 && cLeftY >= -0.08) {
          cLeftY = 0;
        }
        if (cLeftX <= 0.08 && cLeftX >= -0.08) {
          cLeftX = 0;
        }
        drive.curveDrive(dLeftY + cLeftY, -dLeftX - cLeftX);
      }
    }

    // Shooter Controls
    if (shooter.isTargetLocked()) {
      joystick.setRumble(RumbleType.kLeftRumble, 1);
      joystick.setRumble(RumbleType.kRightRumble, 1);

    } else {
      joystick.setRumble(RumbleType.kLeftRumble, 0);
      joystick.setRumble(RumbleType.kRightRumble, 0);

    }

    if (shooter.isWheelSpeedLocked() && shooter.isTargetLocked()) {
      coJoystick.setRumble(RumbleType.kLeftRumble, 1);
      coJoystick.setRumble(RumbleType.kRightRumble, 1);
    } else {
      coJoystick.setRumble(RumbleType.kLeftRumble, 0);
      coJoystick.setRumble(RumbleType.kRightRumble, 0);
    }
    shooter.doStuff(coJoystick.getRightY());
    if (coJoystick.getLeftTriggerAxis() > 0.8) {
      shooter.aim(AimState.on);
      shooter.spinWheels(WheelSpinState.on);
      if (coJoystick.getRightTriggerAxis() > 0.8) {
        if (coJoystick.getBButton() || coJoystick.getXButton() || coJoystick.getYButton()) {
          shooter.shoot(ShooterState.shootNow);
        } else {
          shooter.shoot(ShooterState.shootWhenReady);
        }
      } else {
        shooter.shoot(ShooterState.off);
      }
    } else {
      shooter.aim(AimState.off);

      if (coJoystick.getRightTriggerAxis() > 0.8) {
        shooter.shoot(ShooterState.shootNow);
        shooter.spinWheels(WheelSpinState.on);
      } else {
        shooter.spinWheels(WheelSpinState.off);
        shooter.shoot(ShooterState.off);
      }
    }

    if (coJoystick.getLeftBumper()) {
      shooter.overrideReverseBallfeed = true;
    } else {
      shooter.overrideReverseBallfeed = false;
    }

    isAdjusting = joystick.getYButton();

    if (coJoystick.getXButton()) {
      shooter.shootFrontAtWall();
      if (coJoystick.getRightTriggerAxis() > 0.8) {
        intake.intakeDown();
      }
    } else if (coJoystick.getBButton()) {
      shooter.shootBackAtWall();
    } else if (coJoystick.getYButton()) {
      shooter.shootSafeZone();
    } else if (coJoystick.getLeftTriggerAxis() > 0.8) {
      shooter.useCamera(true);
      shooter.autoAim(distance, drive.getAverageVelocity());
    } else {
      shooter.stow();
    }

    if (coJoystick.getLeftTriggerAxis() < 0.8 && !isAdjusting) {
      shooter.useCamera(false);
    }

    if (coJoystick.getPOV() == 0) {
      shooter.topSwitchZeroed = ZeroedStatus.notZeroed;
    } else {
      shooter.topSwitchZeroed = ZeroedStatus.zeroed;
    }

    // Intake Controls
    if (joystick.getAButton()) {
      intake.intakeDown();
      intake.roller(1);
      pisTime.stop();
      pisTime.reset();
    } else if (joystick.getBButton()) {
      intake.roller(-1);
    } else if (coJoystick.getRightTriggerAxis() >= 0.8) {
      intake.roller(1);
      pisTime.stop();
      pisTime.reset();
    } else if (joystick.getXButton()) {
      intake.intakeNearDown();
      intake.roller(1);
      pisTime.stop();
      pisTime.reset();
    } else {
      pisTime.start();
      if (pisTime.get() < 0.5) {
        intake.intakeNearUp();
        intake.roller(0);
      } else {
        intake.intakeUp();
        intake.roller(0);
      }
    }


    // Gear shift
    drive.gearShift(joystick.getRightBumper());
    SmartDashboard.putBoolean("GearShift", joystick.getRightBumper());

    // climber.setTilt(climbTiltPosition);
    // climber.setHeight(climbHeightPosition);
    // if (coJoystick.getStartButton()) { //TODO change button
    // climber.counteract(gyro.getPitch());
    // }
    // else {
    // climber.tiltOff();
    // }

    if (coJoystick.getStartButtonPressed()) {
      climber.goToNextStep();
    }

    if (coJoystick.getBackButtonPressed()) {
      if (climber.climbStepNumber == 18) {
        climber.climbStepNumber = 16;
      } else {
        climber.goToPreviousStep();
      }
    }
    climber.climbStage(coJoystick.getRightBumper());
    if (coJoystick.getPOV() == 180) {
      climber.climbStepNumber = -1;
    }

    SmartDashboard.putNumber("Climb step number", climber.climbStepNumber);
    SmartDashboard.putBoolean("A", calibrationA);
    SmartDashboard.putBoolean("B", calibrationB);
    SmartDashboard.putBoolean("X", calibrationX);
    SmartDashboard.putBoolean("Y", calibrationY);
    SmartDashboard.putBoolean("L Bumper", calibrationLBumper);
    SmartDashboard.putBoolean("R Bumper", calibrationRBumper);
    SmartDashboard.putBoolean("Back", calibrationBackButton);
    SmartDashboard.putBoolean("Start", calibrationStartButton);

    if (calibrationJoystick.getAButton()) {
      calibrationA = true;
      calibrateInterpolationIndex = 0;
    } else {
      calibrationA = false;
    }

    if (calibrationJoystick.getBButton()) {
      calibrationB = true;
      calibrateInterpolationIndex = 1;
    } else {
      calibrationB = false;
    }

    if (calibrationJoystick.getXButton()) {
      calibrationX = true;
      calibrateInterpolationIndex = 2;
    } else {
      calibrationX = false;
    }

    if (calibrationJoystick.getYButton()) {
      calibrationY = true;
      calibrateInterpolationIndex = 3;
    } else {
      calibrationY = false;
    }

    if (calibrationJoystick.getLeftBumper()) {
      calibrationLBumper = true;
      calibrateInterpolationIndex = 0;
    } else {
      calibrationLBumper = false;
    }

    if (calibrationJoystick.getRightBumper()) {
      calibrationRBumper = true;
      calibrateInterpolationIndex = 1;
    } else {
      calibrationRBumper = false;
    }

    if (calibrationJoystick.getBackButton()) {
      calibrationBackButton = true;
      // not in use
    } else {
      calibrationBackButton = false;
    }

    if (calibrationJoystick.getStartButton()) {
      calibrationStartButton = true;
      calibrateInterpolationIndex = 2;
    } else {
      calibrationStartButton = false;
    }

    if (calibrationB || calibrationA || calibrationX || calibrationY) {
      if (calibrationJoystick.getPOV() == 0 && !debounceCalibration) {
        shooter.autoAimCalc.tilt.values[calibrateInterpolationIndex][1] += 0.5;
        debounceCalibration = true;
      } else if (calibrationJoystick.getPOV() == 180 && !debounceCalibration) {
        shooter.autoAimCalc.tilt.values[calibrateInterpolationIndex][1] -= 0.5;
        debounceCalibration = true;
      } else if (calibrationJoystick.getPOV() == 90 && !debounceCalibration) {
        shooter.autoAimCalc.speedTop.values[calibrateInterpolationIndex][1] += 100;
        shooter.autoAimCalc.speedBot.values[calibrateInterpolationIndex][1] += 100;
        debounceCalibration = true;
      } else if (calibrationJoystick.getPOV() == 270 && !debounceCalibration) {
        shooter.autoAimCalc.speedTop.values[calibrateInterpolationIndex][1] -= 100;
        shooter.autoAimCalc.speedBot.values[calibrateInterpolationIndex][1] -= 100;
        debounceCalibration = true;
      } else if (calibrationJoystick.getPOV() == -1) {
        debounceCalibration = false;
      }
    }

    if (calibrationBackButton || calibrationStartButton || calibrationLBumper || calibrationRBumper) {
      if (calibrationJoystick.getPOV() == 0 && !debounceCalibration) {
        shooter.tiltyFixedval[calibrateInterpolationIndex][1] += 0.5;
        debounceCalibration = true;
      } else if (calibrationJoystick.getPOV() == 180 && !debounceCalibration) {
        shooter.tiltyFixedval[calibrateInterpolationIndex][1] -= 0.5;
        debounceCalibration = true;
      } else if (calibrationJoystick.getPOV() == 90 && !debounceCalibration) {
        shooter.topMotorFixedval[calibrateInterpolationIndex][1] += 100;
        shooter.bottomMotorFixedval[calibrateInterpolationIndex][1] += 100;
        debounceCalibration = true;
      } else if (calibrationJoystick.getPOV() == 270 && !debounceCalibration) {
        shooter.topMotorFixedval[calibrateInterpolationIndex][1] -= 100;
        shooter.bottomMotorFixedval[calibrateInterpolationIndex][1] -= 100;
        debounceCalibration = true;
      } else if (calibrationJoystick.getPOV() == -1) {
        debounceCalibration = false;
      }
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    DataLogManager.getLog().flush();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function is called once w
   * hen test mode is enabled.
   */
  @Override
  public void testInit() {
    pneu.enableCompressorAnalog(60, 110);

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("Pressure PSI", pneu.getPressure(0));
    intake.intakeUp();

  }
}
