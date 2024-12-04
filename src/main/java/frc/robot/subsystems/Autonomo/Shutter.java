package frc.robot.subsystems.Autonomo;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class Shutter extends Command {
    private final Intake intake;
    private final Shooter shooter;

    // Construtor que aceita o subsistema Shooter
    public Shutter(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(shooter); // Declara que o comando requer o Shooter
        addRequirements(intake); // Declara que o comando requer o Intake
    }

    public void setIntake(){
        intake.periodic();
    }

    // public void setDisparo(){
    //     shooter.();
    // }


}
