
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Pickup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ShooterSubsystem;
import frc.robot.subsystems.MechanicalParts.lilElevatorConveyerBeltThingy;


public class ShootMid extends CommandBase {

    private final ShooterSubsystem s_Shooter;
    private final lilElevatorConveyerBeltThingy s_lilElevatorConveyerBeltThingy;

    public PickupPosition(ShooterSubsystem s_Shooter, lilElevatorConveyerBeltThingy s_lilElevatorConveyerBeltThingy) {
        addRequirements(s_Shooter, s_lilElevatorConveyerBeltThingy);
        this.s_Shooter = s_Shooter;
        this.lilElevatorConveyerBeltThingy = lilElevatorConveyerBeltThingy;
    }

    @Override
    public void initialize() {
        s_Shooter.setMid();
        lilElevatorConveyerBeltThingy.shoot();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return lilElevatorConveyerBeltThingy.isDoneWithStuff()
    }
}
