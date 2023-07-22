package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Stow;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.WristSubsystem;

public class StowPosition extends CommandBase {

    private final WristSubsystem s_wrist;

    public StowPosition(
        WristSubsystem s_ClawSubsystem) {
        addRequirements(s_ClawSubsystem);
        this.s_wrist = s_ClawSubsystem;
    }

    @Override
    public void execute() {
        RobotContainer.setCurrentState(RobotContainer.State.Stow);
        s_wrist.setDegrees(Stow.ClawPosition);
        if (!s_wrist.atTargetAngle()) return;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return s_wrist.atTargetAngle();
    }
}
