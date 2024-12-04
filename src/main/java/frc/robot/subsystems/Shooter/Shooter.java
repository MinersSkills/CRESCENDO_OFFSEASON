package frc.robot.subsystems.Shooter;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDBlinker;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Shooter extends SubsystemBase {

    private final CANSparkMax shutterMotor1;
    private final CANSparkMax shutterMotor3;
    private CANSparkMax clawMotor;
    private final Timer timer;
    private final Limelight limelight;
    private final LEDBlinker ledBlinker;
    private RelativeEncoder encoder;

    // Pneumatica
    public static Compressor compressor;
    private PneumaticHub pneumaticHub;
    private DoubleSolenoid solenoid;
    private boolean compressorOn = false;
    private boolean solenoidOnOn = false;

    // private final SwerveSubsystem drivebase;

    private double speakerDistance = 0.0;
    private double ampDistance = 0.0;
    private double speakerDistanceX = 0.0;
    private double ampDistanceX = 0.0;
    private double tx = 0.0;
    private double ty = 0.0;
    private int tagID = 0;

    public static boolean isShooting = false;

    boolean CircleButtonPrevState = false;
    boolean CircleButtonAux = false;
    boolean TrianguleButtonPrevState = false;
    boolean TrianguleButtonAux = false;
    boolean CrossButtonPrevState = false;
    boolean CrossButtonAux = false;
    double l2Axis = 0.0;
    double r2Axis = 0.0;

    boolean R1PrevState = false;
    boolean R1AUX = false;

    // private SwerveSubsystem drivebase;

    public static double adjusted = 0.0;
    public static double adjustedR = 0.0;
    public static double adjustedX = 0.0;
    // Declaração fora do método
    private double previousHeadingError = 0.0; // Inicializado com 0.0

    public Shooter(int motorID1, int motorID3, SwerveSubsystem drivebase) {
        this.shutterMotor1 = new CANSparkMax(motorID1, MotorType.kBrushless);
        this.shutterMotor3 = new CANSparkMax(motorID3, MotorType.kBrushless);
        this.limelight = new Limelight();
        this.ledBlinker = new LEDBlinker();
        // this.drivebase = drivebase;
        this.timer = new Timer();
        clawMotor = new CANSparkMax(31, MotorType.kBrushless);
        encoder = clawMotor.getEncoder();

        // Parte pneumatica
        pneumaticHub = new PneumaticHub(30);
        compressor = new Compressor(30, PneumaticsModuleType.REVPH);
        solenoid = pneumaticHub.makeDoubleSolenoid(0, 2);

    }

    @Override
    public void periodic() {
        updateLimelightData();
        // adjustPosition();
        updateDashboard();
        // turnCompressorOn();
        setGarra();
        // turnSolenoidOn();
        // setAMP();
        setDisparo3();
        setDisparo4();
        // switchAMP();
    }

    private void updateLimelightData() {
        double[] values = limelight.getAprilTagValues();
        double[] distances = limelight.calculateDistance(values[2]);

        tx = values[1];
        ty = values[2];
        tagID = (int) values[0];
        speakerDistance = distances[0];
        ampDistance = distances[1];
        speakerDistanceX = values[1];
        ampDistanceX = values[1];
    }

    // private void handleShooting() {
    // if (tagID == 4) {
    // processShooting(Constants.ps4Controller.getHID().getTriangleButton(), true);
    // } else if (tagID == 2) {
    // processShooting(Constants.ps4Controller.getHID().getCircleButton(), false);
    // }
    // }

    // public void switchAMP() {

    //     boolean R1 = Constants.ps4Controller.getHID().getR1Button();
    //     System.out.println(R1);

    //     if (R1) {
    //         R1AUX = !R1AUX;
    //         if (R1AUX) {
    //             System.out.println("Função AMP chamada");
    //             setAMP();
    //         } else {
    //             System.out.println("Função Garra chamada");
    //             setGarra();
    //         }
    //     }
    // }

    // SetAMP
//     public void setAMP() {
//         boolean crossButton = Constants.ps4Controller.getHID().getCrossButton();
//         boolean squareButton = Constants.ps4Controller.getHID().getSquareButton();

//         double currentPosition = encoder.getPosition();

//         System.out.println("Posição atual: " + currentPosition);

//         // Lógica para subir a garra (Cross Button)
//         if (crossButton) {

//             if (currentPosition <= 49) {
//                 clawMotor.set(0.4);

//                 if (currentPosition >= 49) {
//                     clawMotor.disable();
//                     // clawMotor.getEncoder().setPosition(49);
//                     System.out.println("Garra subiu até a posição desejada.");
//                 }
//             }

//         } else if (squareButton) {
//             if (currentPosition > 0) {
//                 clawMotor.set(-0.4);
//                 if(currentPosition <= 1) {
//                     clawMotor.disable();
//                     System.out.println("Garra desceu até a posição desejada.");
//                 }
//         } 
//     }
// }

    // Disparo ID3
    public void setDisparo3() {
        // Frontal

        boolean circuleButton = Constants.ps4Controller.getHID().getCircleButton();

        if (circuleButton && !CircleButtonPrevState) {
            CircleButtonAux = !CircleButtonAux;
        }
        CircleButtonPrevState = circuleButton;

        if (CircleButtonAux) {
            System.out.println("Selo ativido!");
            if (Constants.sensor.get()) {
                isShooting = true;
                ledBlinker.LedAzul();
                // clawMotor.set(1);
                startShooting(0.3, 0.65, 7);
                System.out.println("Motores ligados");
            } else {
                ledBlinker.LedVermelho();
                stopShooting();
                // clawMotor.set(0);,
                isShooting = false;
                CircleButtonAux = false;
                
            }
        }
    }

    public void downAMP(){
        
        clawMotor.set(-0.4);
        // if (timer.get() >= 0.4) {
        //     clawMotor.set(0);
        // }
        Timer.delay(0.5);
        clawMotor.set(0);
    }

    // Disparo ID4
    public void setDisparo4() {
        // Frontal
        // if (speakerDistance > 4.0 && speakerDistance < 4.6 && speakerDistanceX <
        // 20.00 && speakerDistanceX >= 0.0) {

        boolean TrianguleButton = Constants.ps4Controller.getHID().getTriangleButton();

        if (TrianguleButton && !TrianguleButtonPrevState) {
            TrianguleButtonAux = !TrianguleButtonAux;
        }
        TrianguleButtonPrevState = TrianguleButton;

        if (TrianguleButtonAux) {
            System.out.println("Selo ativido!");
            if (Constants.sensor.get()) {
                isShooting = true;
                ledBlinker.LedAzul();
                startShooting(1.0, 1.0, 1.0);
                System.out.println("Motores ligados");
            } else {
                ledBlinker.LedVermelho();
                stopShooting();
                isShooting = false;
                TrianguleButtonAux = false;
            }
        }
        // }
    }

    // private void adjustPosition() {

    // double KpRotation = 0.022; // Ganho para rotação
    // double kpLateral = 0.01; // Ganho para lateral
    // double kpDistancia = 0.02; // Ganho para lateral
    // double KdRotation = 0.01; // Ganho para rotação
    // double minCommandRotation = 0.05;

    // double distancia_atual = ampDistance;

    // boolean crossButton = Constants.ps4Controller.getHID().getCrossButton();

    // if (crossButton && !CrossButtonPrevState) {
    // CrossButtonAux = !CrossButtonAux;
    // }
    // CrossButtonPrevState = crossButton;

    // if (CrossButtonAux) {

    // double deltaError = tx - previousHeadingError; // Diferença de erro
    // (derivada)
    // previousHeadingError = tx; // Atualiza o erro anterior para a próxima
    // iteração

    // if (Math.abs(tx) > 5.0) {

    // double derivComponent = KdRotation * deltaError; // Termo derivativo (baseado
    // na mudança de erro);

    // // Erro positivo: girar para a esquerda
    // adjustedR = KpRotation * tx + derivComponent;

    // } else if (Math.abs(tx) > 1.0) {

    // adjustedX = kpLateral * tx;

    // } else {
    // adjustedR = 0.0;
    // adjustedX = 0.0;
    // CrossButtonAux = false;
    // }

    // }

    // boolean TrianguleButton =
    // Constants.ps4Controller.getHID().getTriangleButton();

    // if (TrianguleButton && !TrianguleButtonPrevState) {
    // TrianguleButtonAux = !TrianguleButtonAux;
    // }
    // TrianguleButtonPrevState = TrianguleButton;

    // if (TrianguleButtonAux) {

    // double erro_distancia = ampDistance - 2.6;
    // double ajuste_posicao = kpDistancia * erro_distancia;

    // System.out.println("Velocidade: " + ajuste_posicao);

    // // adjusted = ajuste_posicao;
    // }
    // }

    private void startShooting(double motor1Speed, double motor3Speed, double motor2SpeedDelay) {
        shutterMotor1.set(motor1Speed);
        shutterMotor3.set(motor3Speed);
        timer.start();
        if (timer.get() > 0.3) {
            Intake.shutterMotor.set(motor2SpeedDelay);
        }
    }

    private void stopShooting() {
        shutterMotor1.stopMotor();
        Intake.shutterMotor.stopMotor();
        shutterMotor3.stopMotor();
        timer.stop();
        timer.reset();
    }

    // private void turnCompressorOn() {
    // if (Constants.ps4Controller.getHID().getCrossButtonPressed()) {
    // compressorOn = !compressorOn;
    // if (compressorOn) {
    // compressor.enableDigital();
    // System.out.println("Compress on");
    // } else {
    // compressor.disable();
    // System.out.println("Compress off");
    // }
    // }

    // }

    private void turnSolenoidOn() {
        if (Constants.ps4Controller.getHID().getSquareButtonPressed()) {
            solenoidOnOn = !solenoidOnOn;
            if (solenoidOnOn) {
                solenoid.set(Value.kForward);
                System.out.println("Solenoid forward");
            } else {
                solenoid.set(Value.kReverse);
                System.out.println("Solenoid back");
            }
        }
    }

    // // Sobe e Desce braçoAMP
    public void setGarra() {
        // Lógica para controle dos motores de garra com prioridade para R2
        r2Axis = Constants.ps4Controller.getR2Axis();
        l2Axis = Constants.ps4Controller.getL2Axis();

        // Se o R2 estiver sendo acionado (tem prioridade)
        if (r2Axis > 0) {
            if (r2Axis < 0) {
                clawMotor.disable();
            } else {
                // Caso contrário, controle o clawMotor direito com o eixo R2
                clawMotor.set(Constants.ps4Controller.getRawAxis(4) / 2 );
                System.out.println(clawMotor.getEncoder().getPosition());
            }
        } else if (l2Axis > 0) { // Se R2 não estiver sendo acionado, verifique o L2
            if (l2Axis < 0) {
                clawMotor.disable();
            } else {
                // Caso contrário, controle o clawMotor direito com o eixo L2
                clawMotor.set(-Constants.ps4Controller.getRawAxis(3) / 2); // Se precisar controlar o
                System.out.println(clawMotor.getEncoder().getPosition());

            }
        } else { // Se nenhum dos eixos estiver sendo acionado
            clawMotor.disable();
        }

    }

    private void updateDashboard() {
        SmartDashboard.putNumber("Distancia do SPK: ", speakerDistance);
        SmartDashboard.putNumber("Distancia X do SPK: ", speakerDistanceX);

        SmartDashboard.putNumber("Distancia do AMP: ", ampDistance);
        SmartDashboard.putNumber("Distancia X do AMP: ", ampDistanceX);

        SmartDashboard.putBoolean("", Constants.sensor.get());
    }
}