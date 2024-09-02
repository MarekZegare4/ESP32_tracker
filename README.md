

# Work plan for (insert cool project name here)

- [Things to do](#things-to-do)
    - [Hardware](#hardware)
    - [Software](#software)
- [Menu structure](#menu-structure)

## Things to do

### Hardware

- [ ] Finish designign top part
    - [ ] Wio e5 mini case
    - [ ] Main axis connection
    - [ ] RX holder

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

| Date   | Time [h] | Things done              |
|--------|----------|--------------------------|
|2.09.24 |    6     |work plan, menu framwework|
|3.09.24 |          |                          |

## Sources

Menu framework
- https://forum.arduino.cc/t/creating-a-menu-system/896007/2