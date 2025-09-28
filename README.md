# ScopetrexVGA
 Proof of concept Pi Pico2 based X-Y oscilloscope with Z (blank) inputs. Allows display the image from the [Scopetrex](https://github.com/schlae/scopetrex) board on a regular VGA (DVI/HDMI) monitor without an analog XY dual-beam oscilloscope or vector monitor. Current status: it's working but not ideal but it's better than nothing.

## Hardware
 Pico 2 was chosen due to its improved ADC performance compared to the first Pico, as well as the larger amount of memory required for storing samples and a second framebuffer.
### Analog input stage
 The Scoptrex board outputs X/Y signal levels from -5 volts to +5 volts, so a small hardware analog frontend is needed to convert the levels to acceptable Pi Pico voltages from 0 to 3.3 volts. I used the design almost unchanged from the project [scoppy](https://github.com/fhdm-dev/scoppy?tab=readme-ov-file#measuring-different-voltage-ranges-oscilloscope-mode) , specifically this hardware design: https://github.com/fhdm-dev/scoppy/discussions/63
 <img width="1081" height="574" alt="XY_input" src="https://github.com/user-attachments/assets/451cb7ae-d5a9-4bac-ae95-719ce073cf56" />
 The Z output from the Scopetrex board has a maximum signal swing of about 3 volts, so I risked connecting it directly to the Pico. This _may be incorrect_, and a quad op amp like the MCP6004 should be used to prevent possible damage to the Pico input.
 
 Summary:
 - X output from Scopetrex connect over analog front-end to GPIO 26 PiPico2 (ADC0);
 - Y output from Scopetrex connect over analog front-end to GPIO 27 PiPico2 (ADC1);
 - Z output from Scopetrex connect over 100 ohm resistor to GPIO 28 PiPico2 (ADC2);
 - The X and Y outputs should be connected to the front-end board with a shielded (coax) cable. The outputs after the op-amp before the Pico board should also be shielded and as short as possible. The shield is connected to the Pico board's analog ground (AGND).
 - To power the op amp, I used my own 1117 3.3V LDO regulator with a 0.1uf filter capacitor before and after it to reduce power supply noise. Although the difference might not be as significant if you're using power directly from the Pi Pico board's 3V3(OUT) pin.

Example of my front-end prototyping board: </br>
<img width="640" src="https://github.com/user-attachments/assets/79b86c97-f0f1-4ebe-95da-a9e2c8309846">

### VGA output stage
For display image i am using [DispHSTX](https://www.breatharian.eu/hw/disphstx/index_en.html) VGA/DVI RP2350 library from [Miroslav Nemecek](https://github.com/Panda381/DispHSTX), so schematic wiring diagram is exactly the same as shown on DispHSTX web site:
<img src="https://github.com/Panda381/DispHSTX/blob/main/_doc/DispHSTX_diagram.jpg" />
