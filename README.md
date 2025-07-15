# PyTorch-STM32-Gesture-Recognition
Gesture recognition on an STM32 microcontroller using a PyTorch model, trained with data from an ADXL345 accelerometer.

This repository contains a complete project for building a gesture-based controller. The system uses an **STM32 microcontroller** to capture motion data from an **ADXL345 accelerometer**. This data is then used to train a neural network in **PyTorch**.

The trained model is optimized and deployed back to the STM32 for on-device, real-time inference (TinyML). Finally, the recognized gestures are sent over a serial connection to a host computer to control a **Python application**. This project demonstrates an entire workflow, from data collection and model training to final deployment and application.
