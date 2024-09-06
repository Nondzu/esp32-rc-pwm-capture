# esp32-rc-pwm-capture

## 📋 Overview

**`esp32-rc-pwm-capture`** is a PWM signal receiver built for the ESP32, using its RMT module. It is designed to capture and decode PWM signals from RC receivers on multiple GPIO pins. This project provides real-time pulse width logging, with optional LED status indication, making it ideal for custom control systems such as robotics, drones, and other remote-controlled applications.

## 🛠 Hardware Requirements

- **ESP32 Development Board**
- **RC Receiver** or any device that outputs PWM signals
- **LED** (optional for status indication)
- **Wires** for connecting the PWM signal to the ESP32 GPIO pins

## ⚙️ Software Requirements

- **ESP-IDF (Espressif IoT Development Framework)** v5.3.1 or later

## 📝 Setup

1. **Clone the repository**:

    ```bash
    git clone https://github.com/Nondzu/esp32-rc-pwm-capture.git
    cd esp32-rc-pwm-capture
    ```

2. **Set up ESP-IDF**:

    Follow the [ESP-IDF installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) to set up the development environment.

3. **Configure, build, and flash**:

    ```bash
    idf.py set-target esp32
    idf.py build
    idf.py flash
    idf.py monitor
    ```

## 🔌 Wiring

- **PWM Channel 1 (input)**: Connect to **GPIO 2**
- **PWM Channel 2 (input)**: Connect to **GPIO 3**
- **LED**: Optionally connect an LED to **GPIO 21** for status indication.

## 🖥️ Usage

Once flashed, the ESP32 will capture PWM signals on GPIO 2 and GPIO 3, log the pulse width data to the console, and blink the LED to indicate system activity.

### Example Output:

```
I (3000) RC_PWM: Channel 1 - Symbol 0: duration0=1500, duration1=18500
I (3010) RC_PWM: Channel 2 - Symbol 0: duration0=1600, duration1=18400
```

## 📜 License

This project is licensed under the MIT License.
