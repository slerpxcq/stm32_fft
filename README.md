# STM32 Audio Visualizer
![](img.jpg)

## Hardware
- STM32F103C8
- SSD1362
    - 256x64 resolution

## Features
- Fast response
- Full LL library (No HAL)
- Full fixed point arithmetic
- Logarithmic scale
- Double buffering
    - No sample loss
- Interpolation methods:
    - Cubic spline 
    - Linear
    - Tabular

## Configurations
[application.h](Core/Inc/application.h)

Field | Description
| ----- | ----- |
```BAR_FALL_SPEED``` | Bar fall speed
```DOT_FALL_SPEED``` | Dot fall speed
```DOT_TTL``` | Dot hold time
```INTERP_METHOD``` | Interpolation method
Sampling frequency | fs = 72MHz / (TIM3 counter period + 1)
