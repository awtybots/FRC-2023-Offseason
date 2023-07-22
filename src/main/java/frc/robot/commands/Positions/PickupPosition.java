package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Pickup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.WristSubsystem;

public class PickupPosition extends CommandBase {

    private final WristSubsystem s_wrist;

    public PickupPosition(
        WristSubsystem s_WristSubsystem) {
        addRequirements(s_WristSubsystem);
        this.s_wrist = s_WristSubsystem;
    }

    @Override
    public void execute() {
        RobotContainer.setCurrentState(RobotContainer.State.Pickup);
        s_wrist.setDegrees(Pickup.ClawPosition);
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
