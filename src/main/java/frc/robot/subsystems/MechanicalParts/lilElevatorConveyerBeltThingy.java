package frc.robot.subsystems.MechanicalParts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
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

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    Color detectedColor;

    public lilElevatorConveyerBeltThingy() {
        detectedColor = m_colorSensor.getColor();

        // sensor = new I2C(I2C.Port.kOnboard, 0x39); // 0x39 is the sensor's i2c address
        // sensor.write(
        //        0x00, 192); // 0b11000000 ... Power on, color sensor on. (page 20 of sensor datasheet)

        mBeltMotor =
                new CANSparkMax(Constants.ElevatorConveyerThing.ConveyerCanID, MotorType.kBrushless);
        mBeltMotor.restoreFactoryDefaults();

        // mTopShooterMotor.setInverted(true);

        mBeltMotor.setSmartCurrentLimit(30);
        mBeltMotor.setInverted(true);

        BeltMotorPidController = mBeltMotor.getPIDController();

        BeltMotorEncoder = mBeltMotor.getEncoder();

        BeltMotorPidController.setP(Constants.ElevatorConveyerThing.kP);
        BeltMotorPidController.setI(Constants.ElevatorConveyerThing.kI);
        BeltMotorPidController.setD(Constants.ElevatorConveyerThing.kD);
        BeltMotorPidController.setFF(Constants.ElevatorConveyerThing.kFF);
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
        // sensor.read(0x16, 1, buffer);
        // return buffer.get(0);
        return (int) (detectedColor.red * 255);
    }

    public int getGreen() {
        // sensor.read(0x18, 1, buffer);
        // return buffer.get(0);
        return (int) (detectedColor.green * 255);
    }

    public int getBlue() {
        // sensor.read(0x1A, 1, buffer);
        // return buffer.get(0);
        return (int) (detectedColor.blue * 255);
    }

    public void shoot() {
        shootingTime =
                150; // 3 seconds of moving at 20ms frames, TODO USE TIMERS SO THE ROBOT WORKS MORE
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

        SmartDashboard.putNumber("Red", getRed());
        SmartDashboard.putNumber("Green", getGreen());
        SmartDashboard.putNumber("Blue", getBlue());

        detectedColor = m_colorSensor.getColor();

        if (shootingTime > 0) {
            BeltMotorPidController.setReference(
                    Constants.ElevatorConveyerThing.bingFastinVelocity, CANSparkMax.ControlType.kVelocity);
        } else if (!IsCubeDetected()) {
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
