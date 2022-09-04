package frc.robot.subsystem;

import java.util.TreeMap;

public class LerpTable<K extends Double, V extends Double> {
    private final TreeMap<K, V> map = new TreeMap<>();

    public void put(K key, V value) { map.put(key, value);
    }

    public Double get(K key) {
        V val = map.get(key);
        if (val == null) {
            K ceilingKey = map.ceilingKey(key);
            K floorKey = map.floorKey(key);

            if (ceilingKey == null && floorKey == null) {
                return null;
            }
            if (ceilingKey == null) {
                return map.get(floorKey).doubleValue();
            }
            if (floorKey == null) {
                return map.get(ceilingKey).doubleValue();
            }
            V floor = map.get(floorKey);
            V ceiling = map.get(ceilingKey);

            return interpolate(floor, ceiling, inverseInterpolate(ceilingKey, key, floorKey));
        } else {
            return val.doubleValue();
        }
    }


    private double interpolate(V val1, V val2, double d) {
        double dydx = val2.doubleValue() - val1.doubleValue();
        return dydx * d + val1.doubleValue();
    }

    private double inverseInterpolate(K up, K q, K down) {
        double upperToLower = up.doubleValue() - down.doubleValue();
        if (upperToLower <= 0) {
            return 0.0;
        }
        double queryToLower = q.doubleValue() - down.doubleValue();
        if (queryToLower <= 0) {
            return 0.0;
        }
        return queryToLower / upperToLower;
    }
}