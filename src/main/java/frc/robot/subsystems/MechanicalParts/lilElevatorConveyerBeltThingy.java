package frc.robot.subsystems.MechanicalParts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.nio.ByteBuffer;

// STRATEGY
// CONSTANTLY MOVE SLOWLY UNTILL COLOR SENSOR DETECTION, THEN WHEN SHOOT COMMAND MOVE FORWARDS FOR A
// SECOND

public class lilElevatorConveyerBeltThingy extends SubsystemBase {
    private CANSparkMax mBeltMotor;

    private final RelativeEncoder BeltMotorEncoder;

    private final SparkMaxPIDController BeltMotorPidController;

    private int shootingTime = 0;

    ByteBuffer buffer = ByteBuffer.allocate(1);
    I2C sensor;

    public lilElevatorConveyerBeltThingy() {
        sensor = new I2C(I2C.Port.kOnboard, 0x39); // 0x39 is the sensor's i2c address
        sensor.write(
                0x00, 192); // 0b11000000 ... Power on, color sensor on. (page 20 of sensor datasheet)

        mBeltMotor =
                new CANSparkMax(Constants.ElevatorConveyerThing.ConveyerCanID, MotorType.kBrushless);
        mBeltMotor.restoreFactoryDefaults();

        // mTopShooterMotor.setInverted(true);

        mBeltMotor.setSmartCurrentLimit(30);

        BeltMotorPidController = mBeltMotor.getPIDController();

        BeltMotorEncoder = mBeltMotor.getEncoder();

        BeltMotorPidController.setP(Constants.ElevatorConveyerThing.kP);
        BeltMotorPidController.setI(Constants.ElevatorConveyerThing.kI);
        BeltMotorPidController.setD(Constants.ElevatorConveyerThing.kD);
    }

    public boolean IsCubeDetected() {
        int closeness =
                (int)
                        (Math.abs(Constants.ElevatorConveyerThing.r - getRed())
                                + Math.abs(Constants.ElevatorConveyerThing.g - getGreen())
                                + Math.abs(Constants.ElevatorConveyerThing.b - getBlue()));
        return (closeness < Constants.ElevatorConveyerThing.WiggleRoom);
    }

    public int getRed() {
        sensor.read(0x16, 1, buffer);
        return buffer.get(0);
    }

    public int getGreen() {
        sensor.read(0x18, 1, buffer);
        return buffer.get(0);
    }

    public int getBlue() {
        sensor.read(0x1A, 1, buffer);
        return buffer.get(0);
    }

    public void shoot() {
        shootingTime =
                250; // five seconds of moving at 20ms frames, TODO USE TIMERS SO THE ROBOT WORKS MORE
        // CONSISTENTLY
    }

    public void stopShooting() {
        shootingTime = -10;
    }

    public boolean isDoneWithStuff() {
        return shootingTime < 0;
    }

    @Override
    public void periodic() {
        if (!IsCubeDetected() || shootingTime > 0) {
            BeltMotorPidController.setReference(
                    Constants.ElevatorConveyerThing.bingChillinVelocity, CANSparkMax.ControlType.kVelocity);
        } else {
            BeltMotorPidController.setReference(0, CANSparkMax.ControlType.kVelocity);
        }

        if (shootingTime > -10) {
            shootingTime -= 1;
        }
    }
}
