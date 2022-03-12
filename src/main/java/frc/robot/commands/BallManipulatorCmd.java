package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallManipulatorSubsystem;

public class BallManipulatorCmd extends CommandBase {
    
    private final BallManipulatorSubsystem ballManipulatorSubsystem;

    private boolean intakeDown = false;
    private boolean bucketUp = false;

    private Supplier<Boolean> intakeToggleButton;
    private Supplier<Boolean> bucketToggleButton;

    private Supplier<Boolean> topIntakeButton;
    private Supplier<Boolean> bottomIntakeButton;

    private Supplier<Double> topIntakeSlider;
    private Supplier<Double> bottomIntakeSlider;

    public BallManipulatorCmd(BallManipulatorSubsystem ballManipulatorSubsystem, 
            Supplier<Boolean> flipperToggleButtonSupplier, 
            Supplier<Boolean> bucketToggleButtonSupplier,
            Supplier<Boolean> topButtonSupplier, 
            Supplier<Boolean> bottomButtonSupplier,
            Supplier<Double> topSliderSupplier,
            Supplier<Double> bottomSliderSupplier) {

        this.intakeToggleButton = flipperToggleButtonSupplier;
        this.bucketToggleButton = bucketToggleButtonSupplier;

        this.topIntakeButton = topButtonSupplier;
        this.bottomIntakeButton = bottomButtonSupplier;

        this.topIntakeSlider = topSliderSupplier;
        this.bottomIntakeSlider = bottomSliderSupplier;

        this.ballManipulatorSubsystem = ballManipulatorSubsystem;
        addRequirements(ballManipulatorSubsystem);

    }

    @Override
    public void initialize() {
        intakeDown = false;
        bucketUp = false;

        ballManipulatorSubsystem.setIntakeFlipper(false);
        ballManipulatorSubsystem.setBucket(false);

        SmartDashboard.putBoolean("Intake is down: ", intakeDown);
        SmartDashboard.putBoolean("Bucket is up: ", bucketUp);
    }
    
    @Override
    public void execute() {
            if(intakeToggleButton.get()){
                intakeDown = !intakeDown;
            }
            ballManipulatorSubsystem.setIntakeFlipper(intakeDown);

            if(bucketToggleButton.get()){
                bucketUp = !bucketUp;
            }
            ballManipulatorSubsystem.setBucket(bucketUp);

            if(topIntakeButton.get()){
                ballManipulatorSubsystem.setTopIntakePower(topIntakeSlider.get());
            }else{
                ballManipulatorSubsystem.setTopIntakePower(0);
            }
        
            if(bottomIntakeButton.get()){
                ballManipulatorSubsystem.setBottomIntakePower(bottomIntakeSlider.get());
            }else{
                ballManipulatorSubsystem.setBottomIntakePower(0);
            }

        SmartDashboard.putBoolean("Intake is down: ", intakeDown);
        SmartDashboard.putBoolean("Bucket is up: ", bucketUp);
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}

