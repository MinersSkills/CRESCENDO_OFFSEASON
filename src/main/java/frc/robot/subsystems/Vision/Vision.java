package frc.robot.subsystems.Vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Vision {

    private UsbCamera camera;
    private CvSink cvSink;
    private CvSource cvSource;
    private Mat frame;
    private Mat hsvFrame;
    private Mat mask;

    // Parâmetros conhecidos para a medição
    private final double KNOWN_DISTANCE = 100.0; // Distância conhecida em cm
    private final double KNOWN_WIDTH = 10.0; // Largura real do objeto em cm
    private final double FOCAL_LENGTH; // Para ser calculado na inicialização

    public Vision() {
        // Inicializa a captura de vídeo
        camera = CameraServer.startAutomaticCapture();
        camera.setResolution(320, 240);
        camera.setFPS(30);

        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        cvSink = CameraServer.getVideo();
        cvSource = CameraServer.putVideo("Processed", 320, 240);

        frame = new Mat();
        hsvFrame = new Mat();
        mask = new Mat();

        // Calcular a distância focal com base na distância e largura conhecidas
        FOCAL_LENGTH = (320 * KNOWN_DISTANCE) / KNOWN_WIDTH;
    }

    public void processImage() {
        cvSink.grabFrame(frame);

        if (!frame.empty()) {
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerOrange = new Scalar(5, 150, 150);
            Scalar upperOrange = new Scalar(15, 255, 255);

            Core.inRange(hsvFrame, lowerOrange, upperOrange, mask);

            // Encontrar contornos no frame
            java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Selecionar o maior contorno (assumindo que é o objeto desejado)
            if (!contours.isEmpty()) {
                MatOfPoint largestContour = contours.stream().max((c1, c2) -> Double.compare(Imgproc.contourArea(c1), Imgproc.contourArea(c2))).get();

                // Obter a bounding box do contorno
                Rect boundingBox = Imgproc.boundingRect(largestContour);

                // Calcular a distância
                double distance = (KNOWN_WIDTH * FOCAL_LENGTH) / boundingBox.width;

                // Mostrar a distância
                System.out.println("Distância ao alvo: " + distance + " cm");

                // Exibir a imagem processada
                Imgproc.rectangle(frame, boundingBox, new Scalar(0, 255, 0), 2);
            }

            cvSource.putFrame(frame);
        }
    }

    public void stop() {
        if (frame != null) {
            frame.release();
        }
        if (hsvFrame != null) {
            hsvFrame.release();
        }
        if (mask != null) {
            mask.release();
        }
    }
}
