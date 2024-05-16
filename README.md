# murata-sensorhal

Android Sensors HAL implementation for Murata sensors.

## Compiling

Run `make` on directory `murata/sch16xx/`
Sensor HAL implementation `SensorHAL.so` is built.

## Testing

A test application is included in `murata/sch16xx/test/`. Build by
running `make`.

Run the test from the directory `murata/sch16xx` with
```
    test/test-murata-hal ./SensorHAL.so
```    
Check that user permissions are sufficient for iio subsystem, run
as root or alter iio device permissions.
