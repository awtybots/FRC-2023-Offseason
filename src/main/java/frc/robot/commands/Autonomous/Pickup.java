package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Positions.Intake.IntakeFromGroundPosition;
import frc.robot.commands.Positions.PickupPostition;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.MechanicalParts.ArmElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.WristSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.IntakeSubsystem;

public class Pickup extends SequentialCommandGroup {

    public Pickup(
            WristSubsystem s_Claw,
            IntakeSubsystem s_Intake,
            boolean isCone) {
        // addRequirements(s_Claw, s_Arm, s_Elevator, s_Intake);

        addCommands(
                new InstantCommand(() -> RobotContainer.setIsCone(isCone)),
                new PickupPostition(s_Claw),
                new AutonIntakeCurrentLimit(s_Intake).withTimeout(0.2),
                new StowPosition(s_Elevator, s_ArmElevator, s_Claw));
    }
}
