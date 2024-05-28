# ECE4160 - Fast Robots (Cornell University SP24)

This is the source code repository for ***ECE4160 - Fast Robots*** by Yiyan Peng.

* ***main_code*** folder:

There are groups of Arduino and Python code files for Lab1~9 (including BLE & Sensor Setup, PID Control, Kalman Filter, Drift Stunts, Mapping).

* ***localization_simulation*** folder:

Bayes filter, path planning simulation and real-robot implementation source code files can be found under the subfolder ***notebooks***.

* ***libraries*** folder:

Source/example code files for the sensors and the Artemis board.

* ***imu_data_analysis*** folder:

Code files for the analysis of IMU data properties (e.g. drift with time).

## Repo File Tree

```console
.
├── README.md
├── imu_data_analysis
│   ├── acc_data_analysis
│   │   ├── acc_fft.ipynb
│   │   └── imu_data_sample.txt
│   └── gyr_data_drift_analysis
│       ├── gyr_data_sample.txt
│       └── gyr_plot.ipynb
├── libraries
│   ├── ArduinoBLE
│   │   ├── CHANGELOG
│   │   ├── LICENSE
│   │   ├── README.md
│   │   ├── docs
│   │   │   ├── api.md
│   │   │   ├── assets
│   │   │   │   └── ble-bulletin-board-model.png
│   │   │   └── readme.md
│   │   ├── examples
│   │   │   ├── Central
│   │   │   │   ├── LedControl
│   │   │   │   │   └── LedControl.ino
│   │   │   │   ├── PeripheralExplorer
│   │   │   │   │   └── PeripheralExplorer.ino
│   │   │   │   ├── Scan
│   │   │   │   │   └── Scan.ino
│   │   │   │   ├── ScanCallback
│   │   │   │   │   └── ScanCallback.ino
│   │   │   │   └── SensorTagButton
│   │   │   │       └── SensorTagButton.ino
│   │   │   └── Peripheral
│   │   │       ├── Advertising
│   │   │       │   ├── EnhancedAdvertising
│   │   │       │   │   └── EnhancedAdvertising.ino
│   │   │       │   └── RawDataAdvertising
│   │   │       │       └── RawDataAdvertising.ino
│   │   │       ├── BatteryMonitor
│   │   │       │   └── BatteryMonitor.ino
│   │   │       ├── ButtonLED
│   │   │       │   └── ButtonLED.ino
│   │   │       ├── CallbackLED
│   │   │       │   └── CallbackLED.ino
│   │   │       ├── EncryptedBatteryMonitor
│   │   │       │   └── EncryptedBatteryMonitor.ino
│   │   │       └── LED
│   │   │           └── LED.ino
│   │   ├── extras
│   │   │   ├── arduino-ble-parser.py
│   │   │   └── test
│   │   │       ├── CMakeLists.txt
│   │   │       ├── external
│   │   │       │   └── catch
│   │   │       │       └── v2.12.1
│   │   │       │           └── include
│   │   │       │               └── catch.hpp
│   │   │       ├── include
│   │   │       │   ├── Arduino.h
│   │   │       │   ├── test_advertising_data
│   │   │       │   │   └── FakeBLELocalDevice.h
│   │   │       │   ├── test_discovered_device
│   │   │       │   │   └── FakeGAP.h
│   │   │       │   └── util
│   │   │       │       ├── Common.h
│   │   │       │       ├── HCIFakeTransport.h
│   │   │       │       ├── Stream.h
│   │   │       │       ├── String.h
│   │   │       │       ├── TestUtil.h
│   │   │       │       └── itoa.h
│   │   │       └── src
│   │   │           ├── Arduino.cpp
│   │   │           ├── test_advertising_data
│   │   │           │   ├── FakeBLELocalDevice.cpp
│   │   │           │   ├── test_advertising_data.cpp
│   │   │           │   ├── test_local_name.cpp
│   │   │           │   ├── test_manufacturer.cpp
│   │   │           │   └── test_service.cpp
│   │   │           ├── test_discovered_device
│   │   │           │   ├── FakeGAP.cpp
│   │   │           │   └── test_discovered_device.cpp
│   │   │           ├── test_main.cpp
│   │   │           ├── test_uuid
│   │   │           │   └── test_uuid.cpp
│   │   │           └── util
│   │   │               ├── Common.cpp
│   │   │               ├── HCIFakeTransport.cpp
│   │   │               ├── String.cpp
│   │   │               ├── TestUtil.cpp
│   │   │               └── itoa.c
│   │   ├── keywords.txt
│   │   ├── library.properties
│   │   └── src
│   │       ├── ArduinoBLE.h
│   │       ├── BLEAdvertisingData.cpp
│   │       ├── BLEAdvertisingData.h
│   │       ├── BLECharacteristic.cpp
│   │       ├── BLECharacteristic.h
│   │       ├── BLEDescriptor.cpp
│   │       ├── BLEDescriptor.h
│   │       ├── BLEDevice.cpp
│   │       ├── BLEDevice.h
│   │       ├── BLEProperty.h
│   │       ├── BLEService.cpp
│   │       ├── BLEService.h
│   │       ├── BLEStringCharacteristic.cpp
│   │       ├── BLEStringCharacteristic.h
│   │       ├── BLETypedCharacteristic.h
│   │       ├── BLETypedCharacteristics.cpp
│   │       ├── BLETypedCharacteristics.h
│   │       ├── local
│   │       │   ├── BLELocalAttribute.cpp
│   │       │   ├── BLELocalAttribute.h
│   │       │   ├── BLELocalCharacteristic.cpp
│   │       │   ├── BLELocalCharacteristic.h
│   │       │   ├── BLELocalDescriptor.cpp
│   │       │   ├── BLELocalDescriptor.h
│   │       │   ├── BLELocalDevice.cpp
│   │       │   ├── BLELocalDevice.h
│   │       │   ├── BLELocalService.cpp
│   │       │   └── BLELocalService.h
│   │       ├── remote
│   │       │   ├── BLERemoteAttribute.cpp
│   │       │   ├── BLERemoteAttribute.h
│   │       │   ├── BLERemoteCharacteristic.cpp
│   │       │   ├── BLERemoteCharacteristic.h
│   │       │   ├── BLERemoteDescriptor.cpp
│   │       │   ├── BLERemoteDescriptor.h
│   │       │   ├── BLERemoteDevice.cpp
│   │       │   ├── BLERemoteDevice.h
│   │       │   ├── BLERemoteService.cpp
│   │       │   └── BLERemoteService.h
│   │       └── utility
│   │           ├── ATT.cpp
│   │           ├── ATT.h
│   │           ├── BLELinkedList.h
│   │           ├── BLEUuid.cpp
│   │           ├── BLEUuid.h
│   │           ├── GAP.cpp
│   │           ├── GAP.h
│   │           ├── GATT.cpp
│   │           ├── GATT.h
│   │           ├── HCI.cpp
│   │           ├── HCI.h
│   │           ├── HCICordioTransport.cpp
│   │           ├── HCICordioTransport.h
│   │           ├── HCITransport.h
│   │           ├── HCIUartTransport.cpp
│   │           ├── HCIUartTransport.h
│   │           ├── HCIVirtualTransport.cpp
│   │           ├── HCIVirtualTransport.h
│   │           ├── HCIVirtualTransportAT.cpp
│   │           ├── HCIVirtualTransportAT.h
│   │           ├── L2CAPSignaling.cpp
│   │           ├── L2CAPSignaling.h
│   │           ├── bitDescriptions.cpp
│   │           ├── bitDescriptions.h
│   │           ├── btct.cpp
│   │           ├── btct.h
│   │           ├── keyDistribution.cpp
│   │           └── keyDistribution.h
│   ├── BasicLinearAlgebra
│   │   ├── BasicLinearAlgebra.h
│   │   ├── ElementStorage.h
│   │   ├── LICENSE
│   │   ├── README.md
│   │   ├── examples
│   │   │   ├── CustomMatrix
│   │   │   │   └── CustomMatrix.ino
│   │   │   ├── HowToUse
│   │   │   │   └── HowToUse.ino
│   │   │   ├── References
│   │   │   │   └── References.ino
│   │   │   ├── SolveLinearEquations
│   │   │   │   └── SolveLinearEquations.ino
│   │   │   └── Tensor
│   │   │       └── Tensor.ino
│   │   ├── impl
│   │   │   ├── BasicLinearAlgebra.h
│   │   │   └── NotSoBasicLinearAlgebra.h
│   │   ├── keywords.txt
│   │   ├── library.properties
│   │   └── test
│   │       ├── Arduino.h
│   │       ├── CMakeLists.txt
│   │       ├── test_arithmetic.cpp
│   │       ├── test_examples.cpp
│   │       └── test_linear_algebra.cpp
│   ├── SparkFun_9DoF_IMU_Breakout_-_ICM_20948_-_Arduino_Library
│   │   ├── CONTRIBUTING.md
│   │   ├── DMP.md
│   │   ├── ISSUE_TEMPLATE.md
│   │   ├── License.md
│   │   ├── README.md
│   │   ├── examples
│   │   │   ├── Arduino
│   │   │   │   ├── Example10_DMP_FastMultipleSensors
│   │   │   │   │   └── Example10_DMP_FastMultipleSensors.ino
│   │   │   │   ├── Example11_DMP_Bias_Save_Restore_ESP32
│   │   │   │   │   └── Example11_DMP_Bias_Save_Restore_ESP32.ino
│   │   │   │   ├── Example1_Basics
│   │   │   │   │   └── Example1_Basics.ino
│   │   │   │   ├── Example2_Advanced
│   │   │   │   │   └── Example2_Advanced.ino
│   │   │   │   ├── Example3_Interrupts
│   │   │   │   │   └── Example3_Interrupts.ino
│   │   │   │   ├── Example4_WakeOnMotion
│   │   │   │   │   └── Example4_WakeOnMotion.ino
│   │   │   │   ├── Example5_DualSPITest
│   │   │   │   │   └── Example5_DualSPITest.ino
│   │   │   │   ├── Example6_DMP_Quat9_Orientation
│   │   │   │   │   └── Example6_DMP_Quat9_Orientation.ino
│   │   │   │   ├── Example7_DMP_Quat6_EulerAngles
│   │   │   │   │   └── Example7_DMP_Quat6_EulerAngles.ino
│   │   │   │   ├── Example8_DMP_RawAccel
│   │   │   │   │   └── Example8_DMP_RawAccel.ino
│   │   │   │   └── Example9_DMP_MultipleSensors
│   │   │   │       └── Example9_DMP_MultipleSensors.ino
│   │   │   └── PortableC
│   │   │       └── Example999_Portable
│   │   │           └── Example999_Portable.ino
│   │   ├── img
│   │   │   └── Contributing.JPG
│   │   ├── keywords.txt
│   │   ├── library.properties
│   │   └── src
│   │       ├── ICM_20948.cpp
│   │       ├── ICM_20948.h
│   │       └── util
│   │           ├── AK09916_ENUMERATIONS.h
│   │           ├── AK09916_REGISTERS.h
│   │           ├── ICM20948_eMD_nucleo_1.0
│   │           │   └── icm20948_img.dmp3a.h_14290
│   │           ├── ICM_20948_C.c
│   │           ├── ICM_20948_C.h
│   │           ├── ICM_20948_DMP.h
│   │           ├── ICM_20948_ENUMERATIONS.h
│   │           ├── ICM_20948_REGISTERS.h
│   │           ├── eMD-SmartMotion-ICM20948-1.1.0-MP
│   │           │   └── icm20948_img.dmp3a.h_14301
│   │           └── icm20948_img.dmp3a.h
│   └── SparkFun_VL53L1X_4m_Laser_Distance_Sensor
│       ├── LICENSE.md
│       ├── README.md
│       ├── examples
│       │   ├── Example1_ReadDistance
│       │   │   └── Example1_ReadDistance.ino
│       │   ├── Example2_SetDistanceMode
│       │   │   └── Example2_SetDistanceMode.ino
│       │   ├── Example3_StatusAndRate
│       │   │   └── Example3_StatusAndRate.ino
│       │   ├── Example4_SetIntermeasurementPeriod
│       │   │   └── Example4_SetIntermeasurementPeriod.ino
│       │   ├── Example5_LCDDemo
│       │   │   └── Example5_LCDDemo.ino
│       │   ├── Example6_ArduinoPlotterOutput
│       │   │   └── Example6_ArduinoPlotterOutput.ino
│       │   ├── Example7_Calibration
│       │   │   └── Example7_Calibration.ino
│       │   ├── Example8_SetGetDetectionThresholds
│       │   │   └── Example8_SetGetDetectionThresholds.ino
│       │   └── Example9_OneShotMeasurement
│       │       └── Example9_OneShotMeasurement.ino
│       ├── keywords.txt
│       ├── library.properties
│       └── src
│           ├── SparkFun_VL53L1X.cpp
│           ├── SparkFun_VL53L1X.h
│           └── st_src
│               ├── ComponentObject.h
│               ├── LICENSE.md
│               ├── RangeSensor.h
│               ├── vl53l1_error_codes.h
│               ├── vl53l1x_class.cpp
│               └── vl53l1x_class.h
├── localization_simulation
│   ├── README.md
│   ├── Traj.py
│   ├── __init__.py
│   ├── __pycache__
│   │   ├── Traj.cpython-310.pyc
│   │   ├── commander.cpython-310.pyc
│   │   ├── localization.cpython-310.pyc
│   │   ├── localization_extras.cpython-310.pyc
│   │   ├── robot.cpython-310.pyc
│   │   └── utils.cpython-310.pyc
│   ├── commander.py
│   ├── config
│   │   ├── plotter.yaml
│   │   └── world.yaml
│   ├── external_lib
│   │   ├── README.md
│   │   ├── __init__.py
│   │   ├── __pycache__
│   │   │   ├── __init__.cpython-310.pyc
│   │   │   ├── box2d_framework.cpython-310.pyc
│   │   │   ├── box2d_settings.cpython-310.pyc
│   │   │   ├── pygame_framework.cpython-310.pyc
│   │   │   ├── pygame_gui.cpython-310.pyc
│   │   │   └── raycasting.cpython-310.pyc
│   │   ├── box2d_framework.py
│   │   ├── box2d_settings.py
│   │   ├── pygame_framework.py
│   │   ├── pygame_gui.py
│   │   └── raycasting.py
│   ├── localization.py
│   ├── localization_extras.py
│   ├── logs
│   │   ├── __init__.py
│   │   ├── ble.log
│   │   ├── commander.log
│   │   ├── demo_notebook.log
│   │   ├── launcher.log
│   │   ├── localization.log
│   │   ├── localization.log.1
│   │   ├── localization.log.2
│   │   ├── localization.log.3
│   │   ├── localization.log.4
│   │   ├── localization_extras.log
│   │   ├── plotter.log
│   │   ├── plotter.log.1
│   │   └── sim.log
│   ├── notebooks
│   │   ├── __init__.py
│   │   ├── __pycache__
│   │   │   ├── base_ble.cpython-310.pyc
│   │   │   ├── ble.cpython-310.pyc
│   │   │   ├── cmd_types.cpython-310.pyc
│   │   │   └── notebook_utils.cpython-310.pyc
│   │   ├── base_ble.py
│   │   ├── ble.py
│   │   ├── cmd_types.py
│   │   ├── connection.yaml
│   │   ├── lab10.ipynb
│   │   ├── lab11_real.ipynb
│   │   ├── lab11_sim.ipynb
│   │   ├── lab12_path_planning.ipynb
│   │   └── notebook_utils.py
│   ├── robot.py
│   ├── src
│   │   ├── __init__.py
│   │   ├── __pycache__
│   │   │   ├── __init__.cpython-310.pyc
│   │   │   ├── launcher.cpython-310.pyc
│   │   │   ├── plotter.cpython-310.pyc
│   │   │   ├── protocol.cpython-310.pyc
│   │   │   └── sim.cpython-310.pyc
│   │   ├── launcher.py
│   │   ├── plotter.py
│   │   ├── protocol.py
│   │   └── sim.py
│   ├── test.py
│   └── utils.py
├── main_code
│   ├── ble_arduino
│   │   ├── BLECStringCharacteristic.h
│   │   ├── EString.h
│   │   ├── RobotCommand.h
│   │   └── ble_arduino.ino
│   └── ble_python
│       ├── __init__.py
│       ├── __pycache__
│       │   ├── base_ble.cpython-311.pyc
│       │   ├── ble.cpython-311.pyc
│       │   ├── cmd_types.cpython-311.pyc
│       │   └── utils.cpython-311.pyc
│       ├── base_ble.py
│       ├── ble.py
│       ├── ble_drift_stunt.ipynb
│       ├── ble_general.ipynb
│       ├── ble_kalman_filter.ipynb
│       ├── ble_mapping.ipynb
│       ├── ble_pid.ipynb
│       ├── cmd_types.py
│       ├── connection.yaml
│       ├── kalman_readings_csv
│       │   ├── time_dist_2024-03-20_12-16-20.csv
│       │   ├── time_dist_2024-03-20_17-12-27.csv
│       │   ├── time_dist_2024-03-20_17-14-47.csv
│       │   ├── time_dist_2024-03-20_17-28-46.csv
│       │   ├── time_dist_2024-03-20_17-40-30.csv
│       │   └── time_pwm_distance_2024-03-20_19-22-01.csv
│       ├── logs
│       │   ├── __init__.py
│       │   ├── ble.log
│       │   ├── ble.log.1
│       │   ├── ble.log.2
│       │   └── ble.log.3
│       ├── mapping_data
│       │   ├── tof1
│       │   │   ├── point1_tof1_1.csv
│       │   │   ├── point1_tof1_2.csv
│       │   │   ├── point2_tof1_1.csv
│       │   │   ├── point2_tof1_2.csv
│       │   │   ├── point3_tof1_1.csv
│       │   │   ├── point3_tof1_2.csv
│       │   │   ├── point4_tof1_1.csv
│       │   │   ├── point4_tof1_2.csv
│       │   │   ├── point5_tof1_1.csv
│       │   │   └── point5_tof1_2.csv
│       │   └── tof2
│       │       ├── point1_tof2.csv
│       │       ├── point2_tof2.csv
│       │       ├── point3_tof2.csv
│       │       ├── point4_tof2.csv
│       │       └── point5_tof2.csv
│       ├── remapping_data
│       │   └── tof1
│       │       ├── p1_1.csv
│       │       ├── p2_1.csv
│       │       ├── p2_2.csv
│       │       ├── p4_1.csv
│       │       └── p4_2.csv
│       └── utils.py
└── mapping_sim_packages.zip

107 directories, 307 files
```
