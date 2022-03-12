package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LifterConstants;

public class LifterSubsystem extends SubsystemBase {
    private final Solenoid leftLifterPiston = new Solenoid(PneumaticsModuleType.CTREPCM, LifterConstants.kLeftLifterPistonPort);
    private final Solenoid rightLifterPiston = new Solenoid(PneumaticsModuleType.CTREPCM, LifterConstants.kRightLifterPistonPort);

    public LifterSubsystem() {
        leftLifterPiston.set(false);
        rightLifterPiston.set(false);
    }

    public void setLifter(boolean lifterUp) {
        leftLifterPiston.set(lifterUp);
        rightLifterPiston.set(lifterUp);
    }

    public boolean getLifterState() {
        return leftLifterPiston.get();
    }
}
