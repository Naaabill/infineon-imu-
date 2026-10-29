[Click here](../README.md) to view the README.

## Design and implementation

The design of this application is minimalistic to get started with code examples on PSOC&trade; Edge MCU devices. All PSOC&trade; Edge E84 MCU applications have a dual-CPU three-project structure to develop code for the CM33 and CM55 cores. The CM33 core has two separate projects for the secure processing environment (SPE) and non-secure processing environment (NSPE). A project folder consists of various subfolders, each denoting a specific aspect of the project. The three project folders are as follows:

**Table 1. Application projects**

Project | Description
--------|------------------------
*proj_cm33_s* | Project for CM33 secure processing environment (SPE)
*proj_cm33_ns* | Project for CM33 non-secure processing environment (NSPE)
*proj_cm55* | CM55 project

<br>

In this code example, at device reset, the secure boot process starts from the ROM boot with the secure enclave (SE) as the root of trust (RoT). From the secure enclave, the boot flow is passed on to the system CPU subsystem where the secure CM33 application starts. After all necessary secure configurations, the flow is passed on to the non-secure CM33 application. Resource initialization for this example is performed by this CM33 non-secure project. It configures the system clocks, pins, clock to peripheral connections, and other platform resources. It then enables the CM55 core using the `Cy_SysEnableCM55()` function and the CM55 core is subsequently put to DeepSleep mode.

The BMI270 motion sensor is a low-power device that provides measurements of 3-axis accelerometer and 3-axis gyroscope data. This sensor is located on the PSOC&trade; Edge Kit baseboard and the PSOC&trade; Edge MCU utilizes the I2C interface to communicate with the sensor and then converts the data into metric units. After conversion, the data is displayed over the UART terminal. To interface with the BMI270 motion sensor, the PSOC&trade; Edge MCU uses the [BMI270-Sensor-API](https://github.com/boschsensortec/BMI270-Sensor-API/tree/v2.86.1) from Bosch Sensortec.

In the CM33 non-secure application, the clocks and system resources are initialized by the BSP initialization function. The retarget-io middleware is configured to use the debug UART and the user LED 1 is initialized. The debug UART prints a message “PSOC Edge MCU: Sensor hub IMU” on the terminal emulator with a confirmation that the current build is configured in data acquisition mode. The onboard KitProg3 acts the USB-UART bridge to create the virtual COM port. 

You may optionally enable sensor fusion functionality for fusing the 6 axis sensor data using BSXlite sensor fusion library to compute the Euler angles and Quaternion vectors. See the [README](../README.md) file on steps to enable the data fusion mode. 
