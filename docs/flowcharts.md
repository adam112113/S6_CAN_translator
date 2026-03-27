# CAN Translator – Software Flowcharts

These diagrams document the firmware running on the **Olimexino STM32F303RCTx**.  
The firmware translates CAN messages from the **EMUS Mini3 BMS** (standard 11-bit IDs)
to the **Heinzmann ECU** (extended 29-bit IDs) at **500 kbps**.

> **How to use in draw.io**  
> Open [draw.io](https://app.diagrams.net), choose *Extras → Edit Diagram*, select
> *Mermaid* from the format dropdown, and paste the code block of any diagram below.

---

## 1 – System Initialisation & Main Loop

```mermaid
flowchart TD
    A([Power-On / Reset]) --> B[HAL_Init\nInit Flash & SysTick]
    B --> C[SystemClock_Config\nHSE 8 MHz × PLL×9 = 72 MHz]
    C --> D[MX_GPIO_Init\nPA1 = Error LED\nPA5 = Activity LED]
    D --> E[MX_CAN_Init\n500 kbps – pass-all mask filter → FIFO1]
    E --> F[HAL_CAN_Start]
    F --> G[Enable CAN_IT_RX_FIFO1_MSG_PENDING interrupt]
    G --> H{Main loop\nwhile 1 — idle}
    H --> H

    style A fill:#4a9,color:#fff
    style H fill:#adf
```

---

## 2 – CAN Receive Interrupt & EMUS → Heinzmann Translation

This is the core translation logic. It fires every time an EMUS Mini3 BMS
message arrives on the CAN bus.

```mermaid
flowchart TD
    IRQ([CAN_RX1_IRQHandler]) --> CB[HAL_CAN_RxFifo1MsgPendingCallback]
    CB --> R[HAL_CAN_GetRxMessage\nread from FIFO1]
    R --> LED[Toggle PA5 – Activity LED]
    LED --> T[now = HAL_GetTick]
    T --> IDE{rxHeader.IDE\n== CAN_ID_STD?}
    IDE -- No\nExtended ID --> IGN([Ignore – return])
    IDE -- Yes\nStandard ID --> SW{Switch on\nStdId}

    SW --> C180[0x180\nVoltage & Current]
    SW --> C181[0x181\nBMS Status]
    SW --> C186[0x186\nCurrent Limits]
    SW --> C187[0x187\nCapacity / SOC / SOH]
    SW --> C188[0x188\nTemperatures]
    SW --> C18A[0x18A\nError Code]
    SW --> DEF([default – ignore])

    %% 0x180 branch
    C180 --> P180[Parse:\nvoltage = bytes 1:0  unit 0.01 V\ncurrent = bytes 3:2  unit 0.1 A]
    P180 --> RT180{now − last_voltage_send\n≥ 10 ms?}
    RT180 -- No --> END([Return])
    RT180 -- Yes --> S180[send_heinzmann_voltage_current\npackV = voltage×10\ntermV = voltage×10\ncurr  = current×10]
    S180 --> UPD180[last_voltage_send = now]
    UPD180 --> END

    %% 0x181 branch
    C181 --> P181[Store emus_status_byte = rxData 0]
    P181 --> RT181{now − last_status_send\n≥ 100 ms?}
    RT181 -- No --> END
    RT181 -- Yes --> S181[send_heinzmann_status\nstatusByte\ndischargeMax\nchargeMax]
    S181 --> UPD181[last_status_send = now]
    UPD181 --> END

    %% 0x186 branch
    C186 --> P186[Store:\nemus_max_discharge_mA = bytes 5:4 × 100\nemus_max_charge_mA  = bytes 7:6 × 100]
    P186 --> END

    %% 0x187 branch
    C187 --> P187[Store:\nemus_remCap  = bytes 1:0\nemus_soc    = byte 2\nemus_soh    = byte 3\nemus_fullCap = bytes 5:4]
    P187 --> RT187{now − last_capacity_send\n≥ 100 ms?}
    RT187 -- No --> END
    RT187 -- Yes --> S187[send_heinzmann_capacity\nremCap / soc / soh\nfullCap / cycles=0]
    S187 --> UPD187[last_capacity_send = now]
    UPD187 --> END

    %% 0x188 branch
    C188 --> P188[Store:\nemus_avgTemp = bytes 1:0\nemus_maxTemp = bytes 3:2\nemus_minTemp = bytes 5:4]
    P188 --> RT188{now − last_temp_send\n≥ 100 ms?}
    RT188 -- No --> END
    RT188 -- Yes --> S188[send_heinzmann_temperature\navgTemp / maxTemp / minTemp]
    S188 --> UPD188[last_temp_send = now]
    UPD188 --> END

    %% 0x18A branch
    C18A --> P18A[Store emus_errorCode = bytes 1:0]
    P18A --> RT18A{now − last_error_send\n≥ 100 ms?}
    RT18A -- No --> END
    RT18A -- Yes --> S18A[send_heinzmann_error\nerrorCode]
    S18A --> UPD18A[last_error_send = now]
    UPD18A --> END

    style IRQ fill:#c55,color:#fff
    style END fill:#adf
    style IGN fill:#aaa,color:#fff
    style DEF fill:#aaa,color:#fff
```

---

## 3 – safe_CAN_send (Guarded Transmission)

All five `send_heinzmann_*` functions route through this helper to prevent
mailbox overflows from locking up the ISR.

```mermaid
flowchart TD
    A([safe_CAN_send called]) --> B[deadline = HAL_GetTick + 10 ms]
    B --> C{Free TX mailbox\navailable?}
    C -- Yes --> D[HAL_CAN_AddTxMessage]
    C -- No --> E{HAL_GetTick\n> deadline?}
    E -- No --> C
    E -- Yes --> F[PA1 Error LED ON\nmailbox congestion]
    F --> G([Return])
    D --> H{Result\n== HAL_OK?}
    H -- Yes --> G
    H -- No --> I[PA1 Error LED ON\ntransmission error]
    I --> G

    style A fill:#4a9,color:#fff
    style G fill:#adf
    style F fill:#e44,color:#fff
    style I fill:#e44,color:#fff
```

---

## 4 – Heinzmann Message Encoding

Each table shows how EMUS Mini3 BMS data is packed into the corresponding
Heinzmann extended-ID CAN frame.

### 4a – Voltage & Current  `0x1AF0FE11`  DLC=6

```mermaid
packet-beta
  0-15: "Pack Voltage (×10, unit 0.001 V)"
  16-31: "Terminal Voltage (×10, unit 0.001 V)"
  32-47: "Current (×10, signed, unit 0.01 A)"
```

### 4b – Capacity / SOC / SOH  `0x1AF0FC11`  DLC=8

```mermaid
packet-beta
  0-15: "Remaining Capacity (mAh)"
  16-23: "SOC (%)"
  24-31: "SOH (%)"
  32-47: "Full Capacity (mAh)"
  48-63: "Charge Cycles"
```

### 4c – BMS Status  `0x1AF0FB11`  DLC=8

```mermaid
packet-beta
  0-7: "Status Byte"
  8-15: "Reserved (0x00)"
  16-47: "Max Discharge Current (mA, uint32)"
  48-63: "Max Charge Current (mA, uint16)"
```

### 4d – Temperatures  `0x1AF0F111`  DLC=8

```mermaid
packet-beta
  0-15: "Average Temperature (0.1 °C, int16)"
  16-31: "Maximum Temperature (0.1 °C, int16)"
  32-47: "Minimum Temperature (0.1 °C, int16)"
  48-63: "Reserved (0x0000)"
```

### 4e – Error Code  `0x1AF0EE11`  DLC=8

```mermaid
packet-beta
  0-15: "Error Code (uint16)"
  16-63: "Reserved (0x000000000000)"
```

---

## 5 – CAN ID & Scaling Reference

```mermaid
flowchart LR
    subgraph EMUS["EMUS Mini3 BMS  (Standard 11-bit IDs, 500 kbps)"]
        E1["0x180 – Voltage 0.01 V / Current 0.1 A"]
        E2["0x181 – Status byte"]
        E3["0x186 – Max discharge & charge current (×100 mA)"]
        E4["0x187 – RemCap / SOC / SOH / FullCap"]
        E5["0x188 – Avg / Max / Min Temperature"]
        E6["0x18A – Error Code"]
    end

    subgraph MCU["STM32F303 CAN Translator"]
        T["HAL_CAN_RxFifo1MsgPendingCallback\nRate-limited forwarding"]
    end

    subgraph HEINZ["Heinzmann ECU  (Extended 29-bit IDs, 500 kbps)"]
        H1["0x1AF0FE11 – Voltage & Current  ≥10 ms"]
        H2["0x1AF0FB11 – Status            ≥100 ms"]
        H3["0x1AF0FC11 – Capacity          ≥100 ms"]
        H4["0x1AF0F111 – Temperatures      ≥100 ms"]
        H5["0x1AF0EE11 – Error Code        ≥100 ms"]
    end

    E1 --> T
    E2 --> T
    E3 --> T
    E4 --> T
    E5 --> T
    E6 --> T

    T --> H1
    T --> H2
    T --> H3
    T --> H4
    T --> H5
```
