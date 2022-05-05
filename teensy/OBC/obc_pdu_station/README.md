
<img src="https://hsfl.github.io/artemis/resources/logos/hsfl.png" width="100"> <img src="https://hsfl.github.io/artemis/resources/logos/uh_manoa.png" width="100">

# Artemis OBC + PDU Stress Testing

### Available PDU Commands
Start off each command by typing `CMD: ` followed by one of the following commands in the table below.

|    **COMMAND**   |               **FUNCTION**               |
|:----------------:|:----------------------------------------:|
| READ INA219 0x__ | Get Current Sensor Data for address 0x__ |
|    GPIO ON ALL   |             Enable all GPIOs             |
|   GPIO OFF ALL   |             Disable all GPIOs            |
|  SW_3V3_1 ENABLE |          Enable 3.3V Switch (1)          |
| SW_3V3_1 DISABLE |          Disable 3.3V Switch (1)         |
|  SW_3V3_2 ENABLE |          Enable 3.3V Switch (2)          |
| SW_3V3_2 DISABLE |          Disable 3.3V Switch (2)         |
|  SW_5V_1 ENABLE  |           Enable 5V Switch (1)           |
|  SW_5V_1 DISABLE |           Disable 5V Switch (1)          |
|  SW_5V_2 ENABLE  |           Enable 5V Switch (2)           |
|  SW_5V_2 DISABLE |           Disable 5V Switch (2)          |
|  SW_5V_3 ENABLE  |           Enable 5V Switch (3)           |
|  SW_5V_3 DISABLE |           Disable 5V Switch (3)          |
|  SW_5V_4 ENABLE  |           Enable 5V Switch (4)           |
|  SW_5V_4 DISABLE |           Disable 5V Switch (4)          |
|   SW_12V ENABLE  |             Enable 12V Switch            |
|  SW_12V DISABLE  |            Disable 12V Switch            |
|   VBATT ENABLE   |           Enable Battery Switch          |
|   VBATT DISABLE  |          Disable Battery Switch          |
|    WDT ENABLE    |           Enable Watchdog Timer          |
|    WDT DISABLE   |          Disable Watchdog Timer          |
|  HBRIDGE1 ENABLE |             Enable HBRIDGE 1             |
| HBRIDGE1 DISABLE |             Disable HBRIDGE 1            |
|  HBRIDGE2 ENABLE |             Enable HBRIDGE 2             |
| HBRIDGE2 DISABLE |             Disable HBRIDGE 2            |
|   BURN1 ENABLE   |            Enable Burn Wire 1            |
|   BURN1 DISABLE  |            Disable Burn Wire 1           |
|   BURN2 ENABLE   |            Enable Burn Wire 2            |
|   BURN2 DISABLE  |            Disable Burn Wire 2           |
|  BURN_5V ENABLE  |        Enable 5V Burn Wire Switch        |
|  BURN_5V DISABLE |        Disable 5V Burn Wire Switch       |
|       FATFS      | Mount and write to SD Card (hello world) |