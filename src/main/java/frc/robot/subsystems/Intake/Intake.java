package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDBlinker;
import frc.robot.subsystems.Shooter.Shooter;

public class Intake extends SubsystemBase {

    private final CANSparkMax intakeMotor;
    public static CANSparkMax shutterMotor;
    private final LEDBlinker ledBlinker;

    private boolean previousL1ButtonState = false;
    private boolean intakeActive = false;

    public Intake(int intakeID, int shutterID) {
        intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);
        shutterMotor = new CANSparkMax(shutterID, MotorType.kBrushless);
        ledBlinker = new LEDBlinker();
    }

    @Override
    public void periodic() {
        controlIntake();
    }

    private void controlIntake() {
        boolean currentL1ButtonState = Constants.ps4Controller.getHID().getL1Button();
                // Toggle intake state when button is pressed
        if (currentL1ButtonState && !previousL1ButtonState) {
            intakeActive = !intakeActive;
        }
        previousL1ButtonState = currentL1ButtonState;

        if (intakeActive) {
            if (Constants.sensor.get()) {
                // Stop intake if sensor is triggered
                ledBlinker.LedVerde();
                stopIntake();
            } else {
                // Continue intake
                intakeMotor.set(-1.0);
                shutterMotor.set(0.30);
                ledBlinker.LedVermelho();
            }
        } else {
            // Stop intake if not active or if shooting
            if (!Shooter.isShooting) {
                stopIntake();
            }
        }
    }

    public void setMotor() {
        if (!Constants.sensor1.get()) {
            shutterMotor.set(-0.30);
            //voltar para -0.15
        } else {
            shutterMotor.stopMotor();
        }
    }

    public void stopIntake() {
        intakeMotor.stopMotor();
        shutterMotor.stopMotor();
        intakeActive = false;
    }

    public void setIntake() {
        intakeMotor.set(-1.0);
        shutterMotor.set(0.30);
    }
}
