# RideLink: Off-Grid Group Rider Device 🏍️📡

**RideLink** is an affordable, peer-to-peer communication and navigation system designed specifically to prevent group riders (motorcyclists, off-road enthusiasts, etc.) from getting separated in areas with no cellular service. It leverages **LoRa (Long Range) radio technology** to provide basic directional guidance and status updates, keeping your group together, no matter how remote the ride.

## 🌟 Features

* **Off-Grid Reliability:** Operates independently of cellular networks, GPS, or Wi-Fi. Ideal for remote and off-road environments.
* **Peer-to-Peer Direction:** Displays a simple **directional arrow** pointing toward a selected group member.
* **Proximity Indicator:** Uses a color-coded bar/ring based on **Signal Strength (RSSI)** to show distance:
    * 🟢 **Green:** Close proximity
    * 🟡 **Yellow:** Moderate distance
    * 🔴 **Red:** Far away
* **Pre-set Messaging:** Send quick, one-touch status updates like "Wait," "Go," "Stop," or "Help."
* **Low-Cost & Simple:** Designed to be affordable and easy to use, featuring a small screen and minimal buttons.

---

## 🚧 The Problem: Why RideLink Exists

It's an all-too-common experience for group riders to get separated due to traffic, different speeds, or unexpected stops—especially in areas with unreliable or non-existent cell service.

Current solutions fall short:

* **Cell Phones:** Fail completely without reception.
* **High-End GPS Devices:** Are expensive, and often don't solve the peer-to-peer communication problem.

**RideLink fills this gap** by providing a safety net: a reliable, basic directional and communication tool that ensures everyone can easily find their way back to the group.

---

## 🛠️ Technology & Hardware (Planned)

The core of the RideLink system is based on low-power, compact electronics:

| Component | Purpose |
| :--- | :--- |
| **LoRa Radio Module** | Handles all long-range, peer-to-peer data transmission. |
| **Microcontroller (MCU)** | Processes signals, manages the display, and controls the UI. |
| **Small On-Board Screen** | Displays distance color code, directional arrow, and messages. |
| **Directional Antenna** | (Proposed) For achieving basic directional readings by comparing signal strength. |
| **Push Buttons** | For cycling through contacts and sending pre-set messages. |

### 🧭 How Location Tracking Works

RideLink deliberately avoids using expensive or complex GPS modules. Instead, it estimates the relative position of other riders using the **Received Signal Strength Indicator (RSSI)** of the LoRa transmissions.

* **Distance Estimation:** RSSI is translated directly into the color-coded proximity indicator. A stronger signal (closer) means a greener color.
* **Directional Guidance:** Directional information will be derived by comparing the RSSI from different angles or antennas to determine the **general** bearing of the strongest signal from the selected rider.

---

## 🚀 Getting Started (Future)

This section will be updated with setup instructions and schematics as development progresses.

### To Do List:

- [ ] Select initial MCU and LoRa module (e.g., ESP32/SX1276).
- [ ] Develop initial firmware for peer-to-peer connection.
- [ ] Implement basic RSSI-to-Color-Code logic.
- [ ] Design and test the directional logic (antenna/RSSI comparison).
- [ ] Create simple display UI and button controls.
- [ ] Design 3D-printable weatherproof enclosure.

---
