package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechanicalParts.ShooterSubsystem;
import frc.robot.subsystems.MechanicalParts.lilElevatorConveyerBeltThingy;

public class ShootFar extends CommandBase {

    private final ShooterSubsystem s_Shooter;
    private final lilElevatorConveyerBeltThingy s_lilElevatorConveyerBeltThingy;

    public ShootFar(
            ShooterSubsystem s_Shooter, lilElevatorConveyerBeltThingy s_lilElevatorConveyerBeltThingy) {
        addRequirements(s_Shooter, s_lilElevatorConveyerBeltThingy);
        this.s_Shooter = s_Shooter;
        this.s_lilElevatorConveyerBeltThingy = s_lilElevatorConveyerBeltThingy;
    }

    @Override
    public void initialize() {
        s_Shooter.setFar();
        s_lilElevatorConveyerBeltThingy.shoot();
    }

    @Override
    public void end(boolean interrupted) {
        s_Shooter.setOff();

    }

    @Override
    public boolean isFinished() {
        return s_lilElevatorConveyerBeltThingy.isDoneWithStuff();
    }
}
