package frc.robot.subsystems.MechanicalParts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax mTopShooterMotor;
    private CANSparkMax mBottomShooterMotor;

    private final RelativeEncoder TopMotorEncoder;
    private final RelativeEncoder BottomMotorEncoder;

    private final SparkMaxPIDController TopMotorPidController;
    private final SparkMaxPIDController BottomMotorPidController;

    public int ShootingLevel = 0; // 0 = not moving, 1 = mid node, 2 = high node

    public ShooterSubsystem() {
        mTopShooterMotor = new CANSparkMax(Constants.Shooter.kTopShooterID, MotorType.kBrushless);
        mTopShooterMotor.restoreFactoryDefaults();

        mBottomShooterMotor = new CANSparkMax(Constants.Shooter.kBottomShooterID, MotorType.kBrushless);
        mBottomShooterMotor.restoreFactoryDefaults();

        // mTopShooterMotor.setInverted(true);

        mTopShooterMotor.setSmartCurrentLimit(30);
        mBottomShooterMotor.setSmartCurrentLimit(30);

        TopMotorPidController = mTopShooterMotor.getPIDController();
        BottomMotorPidController = mBottomShooterMotor.getPIDController();

        TopMotorEncoder = mTopShooterMotor.getEncoder();
        BottomMotorEncoder = mBottomShooterMotor.getEncoder();

        TopMotorPidController.setP(Constants.Shooter.kP);
        TopMotorPidController.setI(Constants.Shooter.kI);
        TopMotorPidController.setD(Constants.Shooter.kD);
        TopMotorPidController.setFF(Constants.Shooter.kFF);

        BottomMotorPidController.setP(Constants.Shooter.kP);
        BottomMotorPidController.setI(Constants.Shooter.kI);
        BottomMotorPidController.setD(Constants.Shooter.kD);
        BottomMotorPidController.setFF(Constants.Shooter.kFF);

    }

    public void setShooter(int level) {
        ShootingLevel = level;
    }

    public void setOff() {
        setShooter(0);
    }

    public void setMid() {
        setShooter(1);
    }

    public void setHigh() {
        setShooter(2);
    }

    @Override
    public void periodic() {
        
        if (ShootingLevel == 0) {
            TopMotorPidController.setReference(0, CANSparkMax.ControlType.kVelocity);

            BottomMotorPidController.setReference(0, CANSparkMax.ControlType.kVelocity);

        } else if (ShootingLevel == 1) {
            TopMotorPidController.setReference(
                    Constants.ShooterPresets.Mid.topWheelSpeed, CANSparkMax.ControlType.kVelocity);

            BottomMotorPidController.setReference(
                    Constants.ShooterPresets.Mid.bottomWheelSpeed, CANSparkMax.ControlType.kVelocity);
        } else if (ShootingLevel == 2) {
            TopMotorPidController.setReference(
                    Constants.ShooterPresets.High.topWheelSpeed, CANSparkMax.ControlType.kVelocity);

            BottomMotorPidController.setReference(
                    Constants.ShooterPresets.High.bottomWheelSpeed, CANSparkMax.ControlType.kVelocity);
        }



        /*
        if (ShootingLevel == 0) {
            mTopShooterMotor.set(0);

            mBottomShooterMotor.set(0);

        } else if (ShootingLevel == 1) {
            mTopShooterMotor.set(Constants.ShooterPresets.Mid.topWheelSpeed);
            mBottomShooterMotor.set(Constants.ShooterPresets.Mid.bottomWheelSpeed);

        } else if (ShootingLevel == 2) {
            mTopShooterMotor.set(Constants.ShooterPresets.High.topWheelSpeed);
            mBottomShooterMotor.set(Constants.ShooterPresets.High.bottomWheelSpeed);
    }
    */
    SmartDashboard.putNumber("TopShooterVelocity",TopMotorEncoder.getVelocity() );
    SmartDashboard.putNumber("BottomShooterVelocity",BottomMotorEncoder.getVelocity() );
    
}}
