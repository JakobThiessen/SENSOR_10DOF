# SENSOR_10DOF

- ACC/Gyro	(BMI160/323)
- ACC/Mag	(LSM303AGR)
- T/P/RH	(BME280)

# Hardware

<p float="left">
  <img src="/hardware/eagle/eagleUp/main_SMD_FET_a.png" width="400" />
  <img src="/hardware/eagle/eagleUp/main_SMD_FET_b.png" width="400" />
</p>



| Top side                |Bottom side              |
|-----------------------:|:-------------------------:|
| ![image info](./hardware/eagle/eagleUp/main_SMD_FET_top.png ) |  ![image info](./hardware/eagle/eagleUp/main_SMD_FET_bot.png )|

# Software

## Driver

### lsm303agr

Standard C platform-independent drivers for MEMS motion and environmental sensors
- https://www.st.com/en/embedded-software/c-driver-mems.html
- https://github.com/STMicroelectronics/x-cube-mems1

Arduino/Grove
- https://github.com/Seeed-Studio/Grove_6Axis_Accelerometer_And_Compass/blob/master/examples/CompensatedCompass.ino


### BMI160/323:

- https://github.com/boschsensortec/BMI160_driver
- https://github.com/boschsensortec/BMI323-Sensor-API



