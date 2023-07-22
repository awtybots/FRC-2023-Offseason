package frc.robot.commands.Autonomous.ShootPiece;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Autonomous.AutonIntakeNoCurrentLimit;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.MechanicalParts.WristSubsystem;
import frc.robot.subsystems.MechanicalParts.IntakeSubsystem;

public class ShootPiece extends SequentialCommandGroup {

    public ShootPiece(
            IntakeSubsystem s_Intake,
            WristSubsystem s_Claw) {
        addRequirements(s_Intake, s_Claw);
        addCommands(
                new Position(s_Claw),
                new WaitCommand(0.3),
                new AutonIntakeNoCurrentLimit(s_Intake).withTimeout(0.3),
                new InstantCommand(() -> s_Intake.intake(0, false)),
                new StowPosition(s_Claw));
    }
}
