package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallManipulatorSubsystem;

public class BucketAuto extends CommandBase {
    
    private final BallManipulatorSubsystem ballManipulatorSubsystem;

    private Timer timer = new Timer();

    public BucketAuto(BallManipulatorSubsystem ballManipulatorSubsystem) {
        this.ballManipulatorSubsystem = ballManipulatorSubsystem;

        addRequirements(ballManipulatorSubsystem);
    }

    @Override
    public void initialize() {
        timer.start();


        ballManipulatorSubsystem.setIntakeFlipper(false);
        ballManipulatorSubsystem.setBucket(false);
    }
    
    @Override
    public void execute() {
        ballManipulatorSubsystem.setBucket(true);
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }

    
}

