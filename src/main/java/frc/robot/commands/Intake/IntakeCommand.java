package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDBlinker;
import frc.robot.subsystems.Intake.Intake;

public class IntakeCommand extends Command {

    private Intake intake;
    private LEDBlinker ledBlinker;

    public IntakeCommand (Intake intake, LEDBlinker ledBlinker){
        this.intake = intake;
        this.ledBlinker = ledBlinker;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        new Thread( ()-> {
            if (Constants.sensor.get()) {
                // Stop intake if sensor is triggered
                ledBlinker.LedVerde();
                intake.stopIntake();
            } else {
                // Continue intake
                intake.setIntake();
                ledBlinker.LedVermelho();
            }

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }).start();
    }
}