

# Work plan for (insert cool project name here)

- [Things to do](#things-to-do)
    - [Hardware](#hardware)
    - [Software](#software)
- [Menu structure](#menu-structure)

## Things to do

### Hardware

- [ ] Finish designign top part
    - [x] Wio e5 mini case
    - [x] Main axis connection
    - [x] RX holder
    - [x] Pan servo mount
- [ ] Test pan and tilt movement

### Software

- [ ] Implement menu system
    - [x] Framwork
- [ ] Populate menus:
    - [ ] [Main screen](#main-screen)
    - [ ] [UAV status](#uav-status)
    - [ ] [Map](#map)
    - [ ] [Settings](#settings)
- [ ] Figure out catchy name for the project

## Menu structure

    .
    ├── Main screen
    ├── UAV actions
    ├── Map
    └── Settings
        ├── System status (cpu, ram usage etc.)
        ├── Language
        │   ├── Polish
        │   └── English
        └── Bridge mode
            ├── WiFi
            │   ├── AP
            │   └── Connect to network
            ├── Bluetooth
            │   └── Name
            ├── USB
            └── NONE

## Details

#### Main screen

#### UAV status

#### Map

#### Settings

## Work tracker

| Date   | Things done              |
|--------|--------------------------|
|2.09.24 |work plan, menu framwework|
|3.09.24 |3d model update, menu     |
|4.09.24 |--------------------------|
|5.09.24 |wio e5 case               |
|6.09.24 |rx mount, pan - tilt assembly    |
|7.09.24 |tilt servo mount          |
|8.09.24 |pan-tilt assembly tweaks  |
|9.09.24 |tilt axel                 |

## Sources

Menu framework
- https://forum.arduino.cc/t/creating-a-menu-system/896007/2