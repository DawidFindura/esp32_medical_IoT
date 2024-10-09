# esp32_medical_IoT project

Project Overview

This project is a remote data acquisition system designed for real-time monitoring of biomedical signals, specifically ECG, heart rate, and blood oxygen saturation (SpO2). The system is built using an ESP32 microcontroller, equipped with the MAX30102 sensor for heart rate and SpO2 measurements and the AD8232 module for ECG signal acquisition. Data from these sensors is processed by the ESP32 and transmitted wirelessly via the MQTT protocol over Wi-Fi. The project is still under development. Next, the collected data will be  displayed in real-time on a desktop application developed using the Qt framework with QCustomPlot for data visualization. 

The system is intended to provide a reliable, cost-effective solution for remote health monitoring applications, such as telemedicine, home health management, and continuous patient monitoring in clinical settings.
Key Features

    Real-time biomedical signal acquisition (ECG, heart rate, SpO2)
    Wireless data transmission using the MQTT protocol over Wi-Fi
    Desktop application for data visualization using Qt and QCustomPlot
    Designed with a layered software architecture for scalability and maintainability
    Developed in C++ within the ESP-IDF environment, leveraging FreeRTOS for task management
    Platform-independent desktop application compatible with Windows, Linux, and macOS

System Components

    ESP32 Microcontroller: The main processing unit, responsible for sensor data acquisition, processing, and wireless communication.
    MAX30102 Sensor: Measures heart rate and SpO2 (oxygen saturation).
    AD8232 ECG Module: Acquires and processes ECG signals.
    MQTT Protocol: Lightweight messaging protocol used for reliable data transfer over Wi-Fi.
    Qt Framework: Cross-platform application development framework used to create the desktop application.
    QCustomPlot: A plotting library for visualizing real-time biomedical data.

How It Works

    Data Acquisition: The MAX30102 and AD8232 sensors acquire heart rate, SpO2, and ECG signals from the user.
    Data Processing: The ESP32 processes the sensor data and prepares it for transmission.
    Wireless Transmission: Processed data is sent to the desktop application via MQTT over a Wi-Fi network.
    Data Visualization: The desktop application displays the incoming data in real-time on interactive graphs using QCustomPlot.

Installation & Setup

    ESP32 Firmware: Program the ESP32 using the ESP-IDF framework.
    Desktop Application: Compile the Qt application to run on your system (Windows, Linux, or macOS).
    MQTT Broker: Set up a local or cloud-based MQTT broker for communication.

Future Improvements

    Data security enhancements (e.g., encryption for MQTT communication)
    Battery optimization for portable applications
    Mobile application development for real-time monitoring on smartphones
    Cloud integration for long-term data storage and analysis

License

This project is open-source and licensed under the MIT License.

Author
[Dawid Findura] – [dawidfindura@gmail.com]
