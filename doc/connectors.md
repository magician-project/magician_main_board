# Magician Main Board Connections

Due to Magician being a research project (with a pending cascade round) and hardware being very difficult to upgrade there has been an effort to make the Magician Main board 
as expandable as possible, supporting a variety of operation modes using the same hardware.

When connecting the main board with the rest of the system, one must always keep in mind:
1) What is the LED COB lights rated voltage
2) Is the LED COB Light Output connected to the Main Board OR the Camera Board ?
3) What is the LED Voltage input and where it is connected (Main Board OR Camera Board) 


## Main Board and Camera Board Connections overview

There are 2x boards, the Main Board and the Camera Board.

The Main Board looks like this and features the following connectors.

![Magician Main Board PCB Connectors](https://github.com/magician-project/magician_main_board/blob/main/doc/connectors.jpg?raw=true) 


The Camera Board looks like this and features the following connectors.

![Magician Camera Board PCB Connectors](https://github.com/magician-project/magician_main_board/blob/main/doc/camera_connector_board.jpg?raw=true) 


## Main Board Setup Scenarios


If you want to use regular non-overvolted lights and S/W Sync of lights to the Camera:

1) You do not need the Camera Board!
2) Connect the LED COBs to the (YELLOW Top Right) Direct LED COB Light output.
3) Connect the 3.3V LED COB input on the Direct LED Voltage Input (PINK color) of the Main Board
4) Start the Magician Mainboard by supplying the "h" byte 


If you want to use LED overvolting and H/W Sync of lights to the Camera:

1) Connect the Camera Board to the Magician Main Board using a cable that ties the GREEN ( on both images above headers ).
2) Connect the LED COBs to the Overvolted LED COB Light output of the Camera Board.
3) Connect the Overvolted LED COB input voltage on the Direct LED Voltage Input (PINK color) of the Camera Board
4) Start the Magician Mainboard by supplying the "i" byte 


## Closer look on the PCBs and Diagrams

![Magician Camera Board PCB Connectors](https://github.com/magician-project/magician_main_board/blob/main/doc/camera_board.png?raw=true) 

![Magician Main Board close-up](https://github.com/magician-project/magician_main_board/blob/main/doc/pcb.jpg?raw=true) 


## Connecting Ethernet support using a W5100

![W5100 Ethernet SPI chip](https://github.com/magician-project/magician_main_board/blob/main/src/arduino/CameraControllerNano/W5100.jpg?raw=true) 

