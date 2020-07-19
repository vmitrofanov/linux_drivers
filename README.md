# Repository summary
This is a Linux drivers repository with my home projects
##### bb_eternet 
Linux Ethernet driver for Beagle Bone Black (BBB). MDIO part is taken partially from mainline.
BBB uses 100 Mbit/s PHY. This driver iperfed 95 Mbit/s for transmission and 93 Mbit/s for receiving.
It's finished now, but some work isn't done: 
- I would add poll controller to have netconsole access
- Refactor Tx queued process
- Implemnt some features that support Adress Lookup Engine (ALE) module

##### fm24_eeprom
Linux driver for a popular EEPROM memory

##### pci_board_x100
Linux driver for a custom PCI board with a build-in Ethernet switch. This is a wrapper for access to a board resources.

##### af_xdp
This is addon to a Synopsys (stm) driver. It allows using Zero-copy at af_xdp socket. 
 *|Standart (64 bits pps)|Zero-copy (64 bits pps)
-|-|-
tx|~100K|~400K
rx|~90K|~300K
fwd|~85K|~260K
