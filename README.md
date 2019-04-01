### CHPC: Cheap Heat Pump Controller v1.0
<b>The CHPC a minimal cost Heat Pump (HP) controller, which can be used as provided, or can be adopted to nearly all use cases due to open source nature.<b>


## Development

State: active

## <b>Applications:</b>
### <b>1. Thermostat.</b>
_Brief_: <b>Precision thermostat.</b> Simple and cheap. Only 1 relay and 1 temperature sensor required. 
_Application examples:_ Room heat control. Chicken coop climate control. Distillation column or Yogurt maker temperature control.

### <b>2. Heat pump (HP) controller.</b>
_Brief_: <b> Heat pump controler.</b> Controller drives HP system components: compressor, Cold and Hot side Circulating Pumps (CP). Protects system from overload, overheat and freezing up. Drives EEV\* to optimize HP running conditions.
_Application examples:_ DIY heat pump system. Repair module for commercial system.
Water heater, house heating systems and same.
_Available protections_: Compressor: cold start or overheat. Discharge and suction lines protection. Short-term power loss. Anti-freeze. Power overload protection. Others.
\* under development

For more information about Heap Pumps look at [Wikipedia about HP](https://en.wikipedia.org/wiki/Heat_pump)
<br>
### <b>Control interfaces:</b>
 <b>None:</b> Target temperature uploaded to board with firmware and cannot be changed. System used as fixed thermostat. Target temperature can be changed later with firmware re-upload.
 <b>0.96 OLED or 1602 LCD screen + buttons:</b> Simple, local screen controlled system. Remote control is not possible.
 <b>Remote computer terminal over RS-485 line. </b> Target temperature and running conditions under remote control. User can get stats from all T sensors. Up to 1.2 kilometer line.\*
 <b> Remote automated control/stats via RS-485.</b> Firmware was written with python scripting in mind (and real scripts at prototype 485 network).
\* RS-485 specification. Hardware test succeeded on 400 meters line.
<br>
### <b>Relays:</b>
### <b>"Thermostat"</b>:
Only 1 Relay: drives electric heater (any) 
### <b>Heat Pump.</b> Capillary tube, TXV, EEV:
5 Relays, drives all you need:
* Compressor (can be used as external relay driver for High Power systems)
* Cold Circulating Pump (CP)
* Hot CP
* Sump Heater (optional, recommended for outdoor HP installations)
* 4-way Valve (support coming up: autumn 2019)

### <b>Temperature sensors:</b>
* Up to 13 temperature sensors can be connected to CHPC to control all processes that you want. 
* Only 1 sensor needed for "Thermostat" or "Heat Pump capillary/TXV" 
* 3 sensors needed for "HP with EEV" (absolute minimum scheme)
<br>


## PCB Design

There are 2 PCB designs: type "F" and type "G".

## Type "F" features:
- **mostly SMD** componens,
- **internal power** source,
- **10** output channels,
- 4 input channels,
- 5 relays: 1 * 16A + 4 * 7A,
- buzzer,
- EEV support,
- 16 bit **i2c ADC: on board**,
- i2c devices: allowed, 
- T sensors: ds18b20 on one lane.

## Type "G" features:
- **no SMD** components,
- **external or board-placed** power source,
- **5** output channels,
- 4 input channels,
- 5 relays: 2 * 16A + 3 * 7A,
- buzzer,
- EEV support,
- 16 bit **i2c ADC: off board**,
- i2c devices: allowed,
- T sensors: ds18b20 on one lane.

# Type "F" PCB
![Type F PCB](./PCB_Type_F.png)

# Type "G" PCB
![Type G PCB](./PCB_Type_G.png)

....

....
Drafts below.

Sensors abbreviations:

| Abbr. | Full name |
| ----- | -------------------- |
| Tae | after evaporator |
| Tbe | before evaporator |
| Ttarget | target |
| Tsump | sump |
| Tci | cold in |
| Tco | cold out |
| Thi | hot in |
| Tho | hot out |
| Tbc | before condenser |
| Tac | after condenser |
| Touter | outer (outdoor) |
| Ts1 | additional sensor1 |
| Ts2 | additional sensor2 |


