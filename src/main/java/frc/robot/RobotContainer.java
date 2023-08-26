// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Autonomous.Balance.Balance;
import frc.robot.commands.DontShoot;
import frc.robot.commands.DriveParts.*;
import frc.robot.commands.ShootFar;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.ShootMid;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MechanicalParts.*;
import frc.robot.subsystems.MechanicalParts.ShooterSubsystem;
// import frc.robot.subsystems.ledutils;
// import frc.robot.subsystems.ledutils.patterens_eneum;
import frc.robot.subsystems.Swerve.Swerve;
import frc.util.AutonManager;
import frc.util.Controller;
import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Autonomous manager import
    private final AutonManager autonManager;

    // The robot's subsystems
    private final Swerve s_Swerve = new Swerve();
    // private LedSubsystem s_Led = new LedSubsystem(0, 72);

    private final LimelightSubsystem Limelight = new LimelightSubsystem();

    private final ShooterSubsystem s_Shooter = new ShooterSubsystem();
    private final lilElevatorConveyerBeltThingy s_lilElevatorConveyerBeltThingy =
            new lilElevatorConveyerBeltThingy(); // i would abreviate it but
    private final intakeSubsystem s_intake = new intakeSubsystem();

    private static boolean resetPosMode = false;
    private static double angleOffset = 0;

    public enum State {
        Stow,
        Pickup,
        Shooting,
        Balance
    }

    private static State currentState = State.Stow;

    // The controllers
    private final Controller driverController = new Controller(0);
    private final Controller operatorController = new Controller(1);

    private final HashMap<String, Command> eventMap = new HashMap<>();

    private final String[] autonChoices =
            new String[] {
                "RightPlaceTaxi",
                "RightPlaceBalance",
                "MiddlePlace",
                "MiddlePlaceBalance",
                "LeftPlaceBalance",
                "LeftPlaceTaxi",
                "DoNothing"
            };

    public final SwerveAutoBuilder autoBuilder =
            new SwerveAutoBuilder(
                    // Pose2d supplier
                    s_Swerve::getPose,
                    // Pose2d consumer, used to reset odometry at the beginning of auto
                    s_Swerve::resetOdometry,
                    // SwerveDriveKinematics
                    Constants.Drivetrain.kDriveKinematics,
                    // PID constants to correct for translation error (used to create the X and Y PID
                    // controllers)
                    new PIDConstants(Constants.Auton.kPXYController, 0.0, 0.0),
                    // PID constants to correct for rotation error (used to create the rotation controller)
                    new PIDConstants(Constants.Auton.kPThetaController, 0.0, 0.0),
                    // Module states consumer used to output to the drive subsystem
                    s_Swerve::setModuleStates,
                    eventMap,
                    // TODO: make sure that the drive team understands that the alliance color thing matters
                    true,
                    s_Swerve);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        autonManager = new AutonManager(Limelight);
        eventAssignment();
        addAutonomousChoices();
        autonManager.displayChoices();
        // Configure the button bindings
        configureButtonBindings();
        // s_Led.ivans_patterns(patterens_eneum.awtybots);
        // SmartDashboard.putBoolean("EmergencyButton", false);
    }

    /**
     * Use this method to define the command or command groups to be run at each event marker key. New
     * event markers can be created in PathPlanner.
     */
    private void eventAssignment() {
        eventMap.put("ShootHighEvent", new ShootHigh(s_Shooter, s_lilElevatorConveyerBeltThingy));
        eventMap.put("Balance", new Balance(s_Swerve));
    }

    // The RightPlacePickupPlaceBalance is : 1 foot from DriverStation blue line (x: 2.16), 6 inches
    // from Right wall (y: 0.76).
    // The
    /** Use this method to add Autonomous paths, displayed with {@link AutonManager} */
    private void addAutonomousChoices() {
        autonManager.addDefaultOption("Do Nothing.", new InstantCommand());

        for (var i = 0; i < autonChoices.length; i++) {
            autonManager.addOption(
                    autonChoices[i],
                    autoBuilder.fullAuto(
                            PathPlanner.loadPathGroup(
                                    autonChoices[i],
                                    new PathConstraints(
                                            Constants.Auton.kMaxSpeedMetersPerSecond,
                                            Constants.Auton.kMaxAccelerationMetersPerSecondSquared))));
        }
    }

    public static boolean getResetPosMode() {
        return resetPosMode;
    }

    public static void setResetPosMode(boolean mode) {
        resetPosMode = mode;
    }

    public static double getAngleOffset() {
        return angleOffset;
    }

    public static void setAngleOffset(double offset) {
        angleOffset = offset;
    }

    public static State getCurrentState() {
        return currentState;
    }

    public static void setCurrentState(State state) {
        currentState = state;
    }

    public void autonResetGyro() {
        s_Swerve.zeroGyro(180);
    }

    private void configureButtonBindings() {
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.

        final int translationAxis = XboxController.Axis.kLeftY.value;
        final int strafeAxis = XboxController.Axis.kLeftX.value;
        final int rotationAxis = XboxController.Axis.kRightX.value;

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(s_Swerve, driverController, translationAxis, strafeAxis, rotationAxis));

        driverController.buttonA.onTrue(new InstantCommand(() -> s_Swerve.toggleSwerveMode()));
        driverController.buttonY.onTrue(new InstantCommand(s_Swerve::zeroGyro));

        operatorController.buttonStart.onTrue(
                new InstantCommand(
                        () -> {
                            setResetPosMode(true);
                        }));
        operatorController.buttonStart.onFalse(
                new InstantCommand(
                        () -> {
                            setResetPosMode(false);
                        }));

        operatorController.leftTrigger.onTrue(
                new InstantCommand(
                        () -> {
                            s_intake.doIntake();
                        }));

        operatorController.leftTrigger.onFalse(
                new InstantCommand(
                        () -> {
                            s_intake.doAntiIntake();
                        }));

        operatorController.dPadRight.onTrue(new ShootMid(s_Shooter, s_lilElevatorConveyerBeltThingy));
        operatorController.dPadUp.onTrue(new ShootHigh(s_Shooter, s_lilElevatorConveyerBeltThingy));
        operatorController.dPadLeft.onTrue(new ShootFar(s_Shooter, s_lilElevatorConveyerBeltThingy));
        operatorController.dPadDown.onTrue(new DontShoot(s_Shooter, s_lilElevatorConveyerBeltThingy));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonManager.getSelected();
        // return autoBuilder.fullAuto(
        //         PathPlanner.loadPathGroup("GyroTest", new PathConstraints(1, 0.5))); // ! Testing
        // only
    }

    public void idleLimelight() {
        Limelight.setPipeline(7);
        Limelight.setMode(0); // pipeline default
    }
}
