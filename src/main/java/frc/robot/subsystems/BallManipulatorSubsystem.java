package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallManipulatorConstants;

public class BallManipulatorSubsystem extends SubsystemBase {
    private final Solenoid leftIntakePiston = new Solenoid(PneumaticsModuleType.CTREPCM, BallManipulatorConstants.kLeftFlipperPistonPort);
    private final Solenoid rightIntakePiston = new Solenoid(PneumaticsModuleType.CTREPCM, BallManipulatorConstants.kRightFlipperPistonPort);

    private final Solenoid leftBucketPiston = new Solenoid(PneumaticsModuleType.CTREPCM, BallManipulatorConstants.kLeftBucketPistonPort);
    private final Solenoid rightBucketPiston = new Solenoid(PneumaticsModuleType.CTREPCM, BallManipulatorConstants.kRightBucketPistonPort);

    // private final CANSparkMax insideTopMotor = new CANSparkMax(BallManipulatorConstants.kInsideTopMotorPort, MotorType.kBrushless);
    private final CANSparkMax TopMotor = new CANSparkMax(BallManipulatorConstants.kTopMotorPort, MotorType.kBrushless);
    // private final CANSparkMax insideBottomMotor =new CANSparkMax(BallManipulatorConstants.kInsideBottomMotorPort, MotorType.kBrushless);
    private final CANSparkMax BottomMotor =new CANSparkMax(BallManipulatorConstants.kBottomMotorPort, MotorType.kBrushless);

    private boolean intakeDown = false;
    private boolean BucketUp = false;

    public BallManipulatorSubsystem() {
        rightIntakePiston.set(false);
        leftIntakePiston.set(false);
        rightBucketPiston.set(false);
        leftBucketPiston.set(false);
        
        intakeDown = false;
        BucketUp = false;
    }

    public void setIntakeFlipper(boolean intakeDown) {
        rightIntakePiston.set(intakeDown);
        leftIntakePiston.set(intakeDown);
        this.intakeDown = intakeDown;
    }

    public void setBucket(boolean bucketUp) {
        rightBucketPiston.set(bucketUp);
        leftBucketPiston.set(bucketUp);
        this.BucketUp = bucketUp;
    }

    public void setTopIntakePower(double power){
        if(!BucketUp){
            TopMotor.set(power);
        }else{
            TopMotor.set(0);
        }
    }

    public void setBottomIntakePower(double power){
        if(intakeDown){
            BottomMotor.set(power);
        }else{
            BottomMotor.set(0);
        }
    }
}
