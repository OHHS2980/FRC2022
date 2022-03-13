package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallManipulatorSubsystem;

public class IntakeAuto extends CommandBase {
    
    private final BallManipulatorSubsystem ballManipulatorSubsystem;

    private Timer timer = new Timer();

    private boolean intakeDown = true;

    private double bottomIntakePower = 0;
    private double topIntakePower = 0;

    public IntakeAuto(BallManipulatorSubsystem ballManipulatorSubsystem, boolean intakeDown, double bottomIntakePower, double topIntakePower) {
        this.ballManipulatorSubsystem = ballManipulatorSubsystem;

        this.intakeDown = intakeDown;
        
        this.bottomIntakePower = bottomIntakePower;
        this.topIntakePower = topIntakePower;

        addRequirements(ballManipulatorSubsystem);
    }

    @Override
    public void initialize() {
    timer.start();

        ballManipulatorSubsystem.setIntakeFlipper(intakeDown);
        ballManipulatorSubsystem.setBucket(false);

        ballManipulatorSubsystem.setBottomIntakePower(bottomIntakePower);
        ballManipulatorSubsystem.setTopIntakePower(topIntakePower);
    }
    
    @Override
    public void execute() {
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
    }

    
}

