### CHPC: Cheap Heat Pump Controller v1.0

## Development

State: active

Discussion: https://www.forumhouse.ru/threads/469103/

## PCB Design

There are 2 PCB designs: type "F" and type "G".

Both designs state: beta, untested.

## Type "F" features:
- **mostly SMD** componens,
- **internal power** source,
- **10** output channels,
- 5 output channels,
- 4 input channels,
- 5 relays: 1 * 16A + 4 * 7A,
- buzzer,
- EEV support,
- 16 bit **i2c ADC: on board**,
- i2c devices: allowed, 
- T sensors: ds18b20 on one lane.

## Type "G" features:
- **no SMD** components,
- **external, board-placed** power source,
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
