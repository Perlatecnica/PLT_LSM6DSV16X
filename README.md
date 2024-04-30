# X-NUCLEO-IKS4A1

The X-NUCLEO-IKS4A1 serves as a supplementary board for motion MEMS and environmental sensors intended for the STM32 Nucleo. It adheres to the Arduino UNO R3 connector layout and revolves around various sensors such as the LSM6DSV16X 3D accelerometer and 3D gyroscope, the LSM6DSO16IS 3D accelerometer and 3D gyroscope equipped with ISPU, the LIS2DUXS12 3D accelerometer, the LIS2MDL 3D magnetometer, the SHT40-AD1B humidity and temperature sensor, the LPS22DF pressure and temperature sensor, and the STTS22H temperature sensor. The X-NUCLEO-IKS4A1 connects to the STM32 microcontroller or Arduino boards via the I²C pin.

## Examples

Numerous examples are available with the X-NUCLEO-IKS4A1 library.
The following are for IMU LSM6DSV16X

- **xNucleo-IKS4A1_LSM6DSV16X_mbedOS_000-DataLogTerminal:** This application shows how to get data from LSM6DSV16X accelerometer and gyroscope and print them on terminal.
- **xNucleo-IKS4A1_LSM6DSV16X_mbedOS_001-SensorFusion:** This application shows how to use LSM6DSV16X Sensor Fusion features for reading quaternions.
- **xNucleo-IKS4A1_LSM6DSV16X_mbedOS_002-6D_Orientation:** This application shows how to use LSM6DSV16X accelerometer to find out the 6D orientation and display data on a hyperterminal.
- **xNucleo-IKS4A1_LSM6DSV16X_mbedOS_003-LSM6DSV16X_Qvar_Polling:** This application shows how to use LSM6DSV16X Qvar features in polling mode.
- **xNucleo-IKS4A1_LSM6DSV16X_mbedOS_004-SingleTap:** This application shows how to detect the single tap event using the LSM6DSV16X accelerometer.
- **xNucleo-IKS4A1_LSM6DSV16X_mbedOS_005-DoubleTap:** This application shows how to detect the double tap event using the LSM6DSV16X accelerometer.
- **xNucleo-IKS4A1_LSM6DSV16X_mbedOS_006-TiltDetection:** This application shows how to detect the tilt event using the LSM6DSV16X accelerometer.
- **xNucleo-IKS4A1_LSM6DSV16X_mbedOS_007-Pedometer:** This application shows how to use LSM6DSV16X accelerometer to count steps.
- **xNucleo-IKS4A1_LSM6DSV16X_mbedOS_008-FreeFallDetection:** This application shows how to detect the free fall event using the LSM6DSV16X accelerometer.
- **xNucleo-IKS4A1_LSM6DSV16X_mbedOS_009-WakeUpDetection:** This application shows how to detect the wake-up event using the LSM6DSV16X accelerometer.
- **xNucleo-IKS4A1_LSM6DSV16X_mbedOS_010-MachineLearningCore:** This application shows how to detect the activity using the LSM6DSV16X Machine Learning Core.
- **xNucleo-IKS4A1_LSM6DS1V6X_mbedOS_011-FIFO_Interrupt:** This application shows how to get accelerometer and gyroscope data from FIFO using interrupt and print them on terminal.
