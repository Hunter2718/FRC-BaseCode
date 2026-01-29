package frc.robot.commands.PositionPiece;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.positionPiece.PositionPieceSubsystem;

public class ConstantMoveToPosition extends Command {
    private PositionPieceSubsystem piece;
    private Supplier<Double> targetRad;
    private int truthEncoderIndex;
    private int truthMotorIndex;

    public ConstantMoveToPosition(
        PositionPieceSubsystem piece,
        Supplier<Double> targetRad,
        int truthEncoderIndex,
        int truthMotorIndex
    ) {
        this.piece = piece;
        this.targetRad = targetRad;
        this.truthEncoderIndex = truthEncoderIndex;
        this.truthMotorIndex = truthMotorIndex;

        addRequirements(piece);
    }

    @Override
    public void execute() {
        double target = targetRad.get();
        double encPos = piece.getEncoderPositionRad(truthEncoderIndex);
        double motorPos = piece.getMotorPositionRad(truthMotorIndex);

        piece.moveToPositionRad(target, encPos, motorPos);
    }

    @Override
    public void end(boolean interrupted) {
        piece.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}