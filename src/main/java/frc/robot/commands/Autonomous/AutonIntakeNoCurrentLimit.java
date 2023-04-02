package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.IntakeSubsystem;

public class AutonIntakeNoCurrentLimit extends CommandBase {

    private final IntakeSubsystem s_intake;

    public AutonIntakeNoCurrentLimit(IntakeSubsystem IntakeSubsystem) {
        addRequirements(IntakeSubsystem);
        this.s_intake = IntakeSubsystem;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        boolean isCone = RobotContainer.getIsCone();
        s_intake.intake(isCone ? 1.0 : -1.0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
