package frc.robot.log;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface Put<T> {

    void put(String path, T put);
    T convert(Object object);

    Put<Double> DOUBLE = new Put<>() {
        @Override
        public void put(String path, Double put) {
            SmartDashboard.putNumber(path, put);
        }

        @Override
        public Double convert(Object object) {
            if (!(object instanceof Double)) throw new IllegalArgumentException("Not a double!");

            return (Double) object;
        }
    };

    Put<String> STRING = new Put<>() {
        @Override
        public void put(String path, String put) {
            SmartDashboard.putString(path, put);
        }

        @Override
        public String convert(Object object) {
            if (!(object instanceof String)) throw new IllegalArgumentException("Not a string!");

            return (String) object;
        }
    };

    Put<Boolean> BOOL = new Put<>() {
        @Override
        public void put(String path, Boolean put) {
            SmartDashboard.putBoolean(path, put);
        }

        @Override
        public Boolean convert(Object object) {
            if (!(object instanceof Boolean)) throw new IllegalArgumentException("Not a bool!");

            return (Boolean) object;
        }
    };

}
