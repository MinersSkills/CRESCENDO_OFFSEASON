package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;


public class LEDBlinker {
    private final Timer timer;
    private boolean ledState;

    public LEDBlinker() {
        timer = new Timer();
        ledState = false; // Estado inicial do LED (desligado)
    }

    public void startBlinking() {
        timer.start();
    }

    public void updateBlinking() {
        if (timer.get() > 0.8) { // Intervalo de piscar (0.5 segundos)
            ledState = !ledState; // Alterna o estado do LED
            timer.reset(); // Reinicia o temporizador
        }

        // Atualiza o estado dos LEDs com base em ledState
        if (ledState) {
            Constants.RED.set(true);
            Constants.GREEN.set(true);
            Constants.BLUE.set(true);
        } else {
            Constants.GREEN.set(false);
        }
    }

    public void stopBlinking() {
        timer.stop();
        timer.reset();
        ledState = false;
        Constants.GREEN.set(false);
    }

    public void updateBlinkingDown() {
        if (timer.get() > 0.8) { // Intervalo de piscar (0.5 segundos)
            ledState = !ledState; // Alterna o estado do LED
            timer.reset(); // Reinicia o temporizador
        }

        // Atualiza o estado dos LEDs com base em ledState
        if (ledState) {
            Constants.RED.set(true);
            Constants.GREEN.set(true);
            Constants.BLUE.set(true);
        } else {
            Constants.RED.set(false);
        }
    }

    public void stopBlinkingDown() {
        timer.stop();
        timer.reset();
        ledState = false;
        Constants.RED.set(false);
    }

    public void LedVermelho(){
        Constants.GREEN.set(true);
        Constants.BLUE.set(true);
        Constants.RED.set(false);
    }

    public void LedVerde(){
        Constants.GREEN.set(false);
        Constants.BLUE.set(true);
        Constants.RED.set(true);
    }

    public void LedAzul(){
        Constants.GREEN.set(true);
        Constants.BLUE.set(false);
        Constants.RED.set(true);
    }
}
