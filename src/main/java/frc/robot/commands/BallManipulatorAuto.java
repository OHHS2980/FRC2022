package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallManipulatorSubsystem;

public class BallManipulatorAuto extends CommandBase {
    
    private final BallManipulatorSubsystem ballManipulatorSubsystem;

    private Timer timer = new Timer();

    public BallManipulatorAuto(BallManipulatorSubsystem ballManipulatorSubsystem) {
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

        if (timer.hasElapsed(1.5)){
            ballManipulatorSubsystem.setBucket(false);
        }
        
        if (timer.hasElapsed(2)){
            ballManipulatorSubsystem.setBucket(true);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3);
    }

    
}

