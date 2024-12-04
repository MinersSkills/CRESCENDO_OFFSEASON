package frc.robot.subsystems.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private final NetworkTable table;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    // Método para obter os valores da AprilTag
    public double[] getAprilTagValues() {
        final NetworkTableEntry tx = table.getEntry("tx");
        final NetworkTableEntry ty = table.getEntry("ty");
        final NetworkTableEntry ta = table.getEntry("ta");
        final NetworkTableEntry tv = table.getEntry("tv");
        final NetworkTableEntry tid = table.getEntry("tid");

        // Array para armazenar os valores
        final double[] data = new double[5];
        data[0] = tid.getDouble(0.0); // Tag ID
        data[1] = tx.getDouble(0.0); // Deslocamento horizontal (tx)
        data[2] = ty.getDouble(0.0); // Deslocamento vertical (ty)
        data[3] = ta.getDouble(0.0); // Área do alvo (ta)
        data[4] = tv.getDouble(0.0); // Validação do alvo (tv)

        return data;
    }

    // Método para calcular a distância baseado em ty
    public double[] calculateDistance(final double ty) {

        // Quantos graus para trás a limelight está rotacionada a partir da vertical
        double limelightMountAngleDegrees = 20.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 11.8;

        // distance from the target to the floor (Speaker)
        double goalHeightInchesSpeaker = 53.88;

        // distance from the target to the floor (AMP)
        double goalHeightInchesAmp = 50.13;

        double angleToGoalDegrees = limelightMountAngleDegrees + ty;

        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        double distanceFromLimelightToGoalInches = (goalHeightInchesSpeaker - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        double distanceFromLimelightToGoalInchesAMP = (goalHeightInchesAmp - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        // Calcula a distância horizontal usando a tangente
        double distanceSPK = distanceFromLimelightToGoalInches * 0.0254;
        double distanceAMP = distanceFromLimelightToGoalInchesAMP * 0.0254;

        // Inverte o sinal para garantir valores positivos
        distanceSPK = Math.abs(distanceSPK);
        distanceAMP = Math.abs(distanceAMP);

        // Retorna um array com as distâncias para os diferentes alvos
        return new double[] { distanceSPK, distanceAMP };
    }
}
