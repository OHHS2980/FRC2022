package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LifterSubsystem;

public class LifterPositionCmd extends CommandBase {
    
    private final LifterSubsystem lifterSubsystem;

    private Supplier<Boolean> buttonPressed;

    private boolean lifterUp;

    public LifterPositionCmd(LifterSubsystem lifterSubsystem, Supplier<Boolean> buttonSupplier) {
        this.buttonPressed = buttonSupplier;
        this.lifterSubsystem = lifterSubsystem;
        addRequirements(lifterSubsystem);
    }

    @Override
    public void initialize() {
        lifterUp = false;
        lifterSubsystem.setLifter(false);
        SmartDashboard.putBoolean("Lifter is up: ", lifterUp);
    }
    
    @Override
    public void execute() {
        if(buttonPressed.get()){
            lifterUp = !lifterUp;
        }
        lifterSubsystem.setLifter(lifterUp);
        SmartDashboard.putBoolean("Lifter is up: ", lifterUp);
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
