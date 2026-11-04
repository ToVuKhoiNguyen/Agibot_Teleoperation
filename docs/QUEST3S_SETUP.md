# Meta Quest 3S WebXR Setup Guide

## Overview
Teleoperation uses WebXR running inside the native Meta Quest Browser. No sideloaded native APK is required.

WebXR APIs mandate a secure context (**HTTPS** and **WSS**).

---

## 1. SSL Certificates
The HTTPS server automatically generates self-signed certificates on startup in `certs/` (`certs/cert.pem` and `certs/key.pem`) if they are missing.

Alternatively, generate them manually:

```bash
mkdir -p certs
openssl req -x509 -newkey rsa:2048 -keyout certs/key.pem -out certs/cert.pem -days 365 -nodes \
    -subj "/C=US/ST=Dev/L=Lab/O=Teleop/CN=0.0.0.0"
```

---

## 2. Network Connection Methods

### Method A: USB Tethering via ADB Reverse (Fastest & Lowest Latency)
Connect Quest 3S to PC via USB-C with developer mode enabled:
```bash
adb reverse tcp:8012 tcp:8012
```
In Meta Quest Browser, navigate to:
```text
https://127.0.0.1:8012?ws=wss://127.0.0.1:8012
```

### Method B: Wi-Fi LAN
Ensure the PC and Quest 3S are on the same 5 GHz Wi-Fi network:
1. Find PC local IP: `hostname -I` (e.g., `192.168.1.100`).
2. In Meta Quest Browser, navigate to:
```text
https://192.168.1.100:8012?ws=wss://192.168.1.100:8012
```

---

## 3. Accepting Self-Signed Certificate in Quest Browser
1. When navigating to the URL, Quest Browser will display a security warning: *"Your connection is not private"*.
2. Click **Advanced**.
3. Click **Proceed to ... (unsafe)**.
4. Click **Enter VR** in the WebXR canvas.

---

## 4. Operator Controls & Safety
* **Hand Tracking**: Ensure hand tracking is enabled in Quest 3S settings.
* **Pinch / Grasp**: Pinch index to thumb to close the A2 hand fingers.
* **Tracking Loss**: If the headset loses tracking or operator removes the headset, the system automatically triggers hold-pose.
* **E-Stop**: Press `q` in the terminal to trigger immediate emergency hold/stop.
* **Reset**: Press `r` in the terminal to reset robot to home position.
