
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ShooterSubsystem;
import frc.robot.subsystems.MechanicalParts.lilElevatorConveyerBeltThingy;


public class DontShoot extends CommandBase {

    private final ShooterSubsystem s_Shooter;
    private final lilElevatorConveyerBeltThingy s_lilElevatorConveyerBeltThingy;

    public DontShoot(ShooterSubsystem s_Shooter, lilElevatorConveyerBeltThingy s_lilElevatorConveyerBeltThingy) {
        addRequirements(s_Shooter, s_lilElevatorConveyerBeltThingy);
        this.s_Shooter = s_Shooter;
        this.s_lilElevatorConveyerBeltThingy = s_lilElevatorConveyerBeltThingy;
    }

    @Override
    public void initialize() {
        s_Shooter.setOff();
        s_lilElevatorConveyerBeltThingy.stopShooting();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return s_lilElevatorConveyerBeltThingy.isDoneWithStuff()
    }
}
