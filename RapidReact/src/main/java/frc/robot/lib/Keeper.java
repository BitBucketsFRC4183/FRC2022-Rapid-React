package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import java.util.HashMap;
import java.util.Map;

public class Keeper {

    private final Map<Class<?>, Object> subsystems = new HashMap<>();

    public <T> void addSubsystem(Class<T> clazz, T subsystem) {
        subsystems.put(clazz, subsystem);
    }

    private final Map<Class<?>[], ScriptCtor<?>> scriptregs = new HashMap<>();

    public void build() {


        scriptregs.forEach((c,s) -> {
            TalonSRX

            for (Class<?> cls : c) {
                if (!subsystems.containsKey(cls)) throw new IllegalStateException("badbadbadbad");
            }


        });

    }



}
