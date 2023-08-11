package frc.robot.subsystems.MechanicalParts;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.lang.Math;
import java.nio.ByteBuffer;

//STRATEGY
//INTAKE WHILE LEFT TRIGGER

public class intakeSubsystem extends SubsystemBase {
    private CANSparkMax mIntakeMotor;

    private final RelativeEncoder IntakeMotorEncoder;

    private final SparkMaxPIDController IntakeMotorPidController;
    private double velocity = 0;

    public intakeSubsystem(){

        mIntakeMotor = new CANSparkMax(Constants.intake.intakeCanID, MotorType.kBrushless);
        mIntakeMotor.restoreFactoryDefaults();
        


        //mTopShooterMotor.setInverted(true);

        mIntakeMotor.setSmartCurrentLimit(30);


        IntakeMotorPidController = mIntakeMotor.getPIDController();

        IntakeMotorEncoder = mIntakeMotor.getEncoder();

        IntakeMotorPidController.setP(Constants.intake.kP);
        IntakeMotorPidController.setI(Constants.intake.kI);
        IntakeMotorPidController.setD(Constants.intake.kD);



    }
    public void doIntake(){
        velocity = Constants.intake.intakeVelocity;
    }

    public void doAntiIntake(){
        velocity = 0;
    }

    @Override
    public void periodic() {
        IntakeMotorPidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }}