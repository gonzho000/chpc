### CHPC: Cheap Heat Pump Controller v1.x
<b>The CHPC a minimal cost Heat Pump (HP) controller, which can be used as provided, or can be adopted to nearly all use cases due to open source nature.</b>
<br><br>

## Real life installation.
Works from ground heat collectors (loops) to radiant in-floor heating system.

 ![Installation example](./m_CHPC_i2.jpg)
 ![Installation example](./m_CHPC_i1.jpg)

Driving:
- EEV, 
- Heat Pump Compressor (1kW electrical power),
- Circulating Pumps,
- Sump Heater.

Temperature sensors installed:
- Before/After Evaporator,
- Cold In/Cold Out,
- Hot In(used as Target)/Hot Out,
- Outdoor temperature,
- Sump.

Controled via both RS-485 and 16x2 display with buttons.

## Changelog
- 13 apr, 2019: EEV support development started.
- 16 apr, 2019: Standalone EEV (no thermostat) with only 2 T sensors written and debugged.
- 30 apr, 2019: HP system updated to CHPC.
- 01 may, 2019: CHPC fully tested and released
- 02 may, 2019: PCB rev.1.3 coming up, main feature: lot of DS18B20 inputs
- 17 may, 2019: <b>PCB 1.3 tested, [assembly instructions added](https://github.com/gonzho000/chpc/wiki/assembly)</b>
<br><br>
## Applications:
| Usage. |	Brief description. | 	Application examples	| Available protections	|
| ---------- | ------------------ | ------------------ | -------------------- |
| 1. Thermostat.	|  Precision thermostat. Simple and cheap. Only 1 relay and 1 temperature sensor required.<br> | Room heat control. Chicken coop climate control. Distillation column. Else. | N/A	|
| 2. Heat pump (HP) control. | Controller drives HP system components: compressor, Cold and Hot side Circulating Pumps (CP). Protects system from overload, overheat and freezing up. Drives EEV to optimize running conditions. | DIY heat pump system. Repair module for commercial system. Water heater, house heating systems and same. | Compressor: cold start or overheat. Discharge and suction lines protection. Short-term power loss. Anti-freeze. Power overload protection. |
| 3. EEV controller. | Only drives EEV, no relays. Requires 2 T sensors. | Upgrade your system from capilary tube to EEV. | Protects from liquid at suction line by design. |

For more information about Heap Pumps look at [Wikipedia about HP](https://en.wikipedia.org/wiki/Heat_pump)
<br><br>
## Features:
- Up to 13 T sensors (see "T sensors abbreviations" for full list)
- 5 relays (Compressor, Hot CP or Air Fun, Cold CP or Air Fun, Sump Heater, 4-way valve)
- 4 inputs
- 5/6 pin EEV connection,
- 1602 display support
- RS485 or Serial(UART 5V) support
- Automatically turns on/of system when heating required
- Takes care of system components health
<br><br>
## Control interfaces:
 <b>None:</b> Target temperature uploaded to board with firmware and cannot be changed. System used as fixed thermostat. Target temperature can be changed later with firmware re-upload.<br>
 <b>0.96 OLED or 1602 LCD screen + buttons:</b> Simple, local screen controlled system. Remote control is not possible.<br>
 <b>Remote computer terminal over RS-485 line. </b> Target temperature and running conditions under remote control. User can get stats from all T sensors. Up to 1.2 kilometer line.\*<br>
 <b> Remote automated control/stats via RS-485.</b> Firmware was written with python scripting in mind (and real scripts at prototype 485 network).<br>
 <b> Both screen + buttons and RS-485.</b> Combination allowed.
 
\* RS-485 specification. Hardware test succeeded on 400 meters line.

Example: day/night setpoint control and data visualisation with "pythonic rs485" way.
![graph example](./m_t_graph_example.png)
<br><br>

## Relays:
### "Thermostat":
Only 1 Relay: drives electric heater (any) 
### "Heat Pump". Capillary tube, TXV, EEV:
5 Relays, drives all you need:
* Compressor (can be used as external relay driver for High Power systems)
* Cold Circulating Pump (CP)
* Hot CP
* Sump Heater (optional, recommended for outdoor HP installations)
* 4-way Valve (support coming up: autumn 2019)
<br><br>
## Temperature sensors:
* Up to 13 temperature sensors can be connected to CHPC to control all processes that you want. 
* Only 1 sensor needed for "Thermostat" or "Heat Pump capillary/TXV" 
* 3 sensors needed for "HP with EEV" (absolute minimum scheme)
<br><br>
## Temperature sensors installation example (medium scheme)
![medium scheme](./m_HeatPump_t_sensors_med.png)
<br><br>
 ## Get your own CHPC:
* download PCB gerber file, *CHPC_v1.x_PCB_Gerber.zip*
* search google [where to order PCB](https://www.google.com/search?q=order+pcb) or make your own at CNC machine
* order electronic components, see BOM (Bill Of Materials) list, *CHPC_v1.x_PCB_BOM.html*
* solder, [assembly instructions here](https://github.com/gonzho000/chpc/wiki/assembly)
* install firmware *CHPC_firmware.ino*
* install CHPC at your system
* enjoy
<br><br>
## T sensors abbreviations:
This abbrevations used in interface during sensors installation procedure

| Abbr. | Full name | Required for |
| ----- | -------------------- | -------------------- |
| Tae | after evaporator | EEV <br>Anti-liquid protection at suction line |
| Tbe | before evaporator | EEV |
| Ttarget | target | Thermostat<br>Thermostat+EEV |
| Tsump | sump | Automatic Sump Heater<br>Sump overheat protection |
| Tci | cold in | Antifreeze protection |
| Tco | cold out | Antifreeze protection |
| Thi | hot in |  Hot CP automatic control   |
| Tho | hot out | Overheat protection     |
| Tbc | before condenser | Discharge overheat protection           |
| Tac | after condenser |          |
| Touter | outer (outdoor) |          |
| Ts1 | additional sensor1 |          |
| Ts2 | additional sensor2 |          |

<br><br>
 ## Photos:<br>
 ![v1.3](./m_PCB_v1.3_noscreen.jpg)
 ![v1.3](./m_PCB_v1.3_screen.jpg)
 ![v1.3](./m_v1.3_PCBdemo.png) 

 
 ## Older revisions and prototypes:<br>
 PCB v1.1
 
 ![proto3](./m_proto3.jpg)
 ![proto3 without screen](./m_proto3_noscreen.jpg)
 
 This is prototype 2 (PCB v1.0).
 1602 is the best choise.
 
 ![proto2](./m_proto2.jpg)
 
 EEV development started here.
 
 ![proto2_EEVdev](./m_proto2_EEVdev.jpg)
 
 ![proto2 PCB](./m_proto2_PCB.jpg)
 
Prototype 1.
History ) But worked well for a season.
 ![proto1](./m_proto1.jpg)

<br><br>

## Author:
gonzho АТ web.de (c) 2018-2019

If you have any comments or questions, please do not hesitate to contact me.
