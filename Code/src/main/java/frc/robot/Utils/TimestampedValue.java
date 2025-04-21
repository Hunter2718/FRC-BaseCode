package frc.robot.Utils;

import java.util.Objects;

public class TimestampedValue<T> {
    public T value;
    public double timestamp;

    public TimestampedValue(T value, double timestamp) {
        this.value = value;
        this.timestamp = timestamp;
    }

    public void update(T value, double timestamp) {
        this.value = value;
        this.timestamp = timestamp;
    }

    @Override
    public String toString() {
        return "Value: " + value + ", Time: " + timestamp;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof TimestampedValue<?> other)) return false;
        return Objects.equals(value, other.value) && timestamp == other.timestamp;
    }

    @Override
    public int hashCode() {
        return Objects.hash(value, timestamp);
    }
}
