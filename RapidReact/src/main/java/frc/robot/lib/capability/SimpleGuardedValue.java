package frc.robot.lib.capability;

public class SimpleGuardedValue<T> implements GuardedValue<T> {

    final EdgeBorrow borrow;
    final T value;

    public SimpleGuardedValue(EdgeBorrow borrow, T value) {
        this.borrow = borrow;
        this.value = value;
    }

    @Override
    public boolean tryAcquire(String reason) {
        return borrow.tryBorrow(reason);
    }

    @Override
    public T acquire() {
        if (!borrow.safe()) throw new IllegalStateException("Tried to borrow a value that was already being borrowed!");

        return value;
    }
}
