package frc.robot.commands.Autonomous.ShootPiece;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Nodes.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.WristSubsystem;

public class Position extends CommandBase {

    private final WristSubsystem s_claw;

    public Position(
            WristSubsystem s_clawSubsystem) {
        addRequirements();
        this.s_claw = s_clawSubsystem;
    }

    @Override
    public void execute() {
        RobotContainer.setCurrentState(RobotContainer.State.Shooting);
        s_claw.setDegrees(Cube.ShootCube.ClawPosition);
    }

    @Override
    public boolean isFinished() {
        return s_claw.atTargetAngle();
    }
}
