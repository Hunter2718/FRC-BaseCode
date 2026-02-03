package frc.robot.commands.PositionPiece;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.positionPiece.PositionPieceSubsystem;
import frc.robot.utils.UnitsUtils;

public class MoveToPosition extends Command {
    private PositionPieceSubsystem piece;
    private Supplier<Double> targetRad;
    private int truthEncoderIndex;
    private int truthMotorIndex;
    private double truthEncoderToPieceGearRatio;

    public MoveToPosition(
        PositionPieceSubsystem piece,
        Supplier<Double> targetRad,
        int truthEncoderIndex,
        int truthMotorIndex,
        double truthEncoderToPieceGearRatio
    ) {
        this.piece = piece;
        this.targetRad = targetRad;
        this.truthEncoderIndex = truthEncoderIndex;
        this.truthMotorIndex = truthMotorIndex;

        this.truthEncoderToPieceGearRatio = truthEncoderToPieceGearRatio;

        addRequirements(piece);
    }

    @Override
    public void execute() {
        double target = targetRad.get();
        double encPos = UnitsUtils.applyGearRatio(piece.getEncoderPositionRad(truthEncoderIndex), truthEncoderToPieceGearRatio);
        double motorPos = piece.getMotorPositionRad(truthMotorIndex);

        piece.moveToPositionRad(target, encPos, motorPos);
    }

    @Override
    public boolean isFinished() {
        return piece.atGoal(truthEncoderIndex);
    }
}
