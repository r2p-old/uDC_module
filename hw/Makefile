# Eagle executable path
EAGLE := /opt/eagle-6.6.0/bin/eagle

# Gerbv PCB image preview parameters - colours, plus resolution
GERBER_IMAGE_RESOLUTION?=1200
BACKGROUND_COLOUR?=000000
HOLES_COLOUR?=000000
SILKSCREEN_COLOUR?=C4C4C4
PADS_COLOUR?=FFDE4E
TOP_SOLDERMASK_COLOUR?=8D2323CC
BOTTOM_SOLDERMASK_COLOUR?=23238D

# Constants
SHELL := /bin/bash
MAKEFILE_PATH := $(abspath $(MAKEFILE_LIST))
PROJECT_NAME := $(notdir $(patsubst %/hw/,%,$(dir $(MAKEFILE_PATH))))
BOARD := $(PROJECT_NAME).brd
SCHEMATIC := $(PROJECT_NAME).sch

all: gerbers doc

debug:
	$(info MAKEFILE_PATH: $(MAKEFILE_PATH))
	$(info PROJECT_NAME: $(PROJECT_NAME))
	$(info BOARD: $(BOARD))
	$(info SCHEMATIC: $(SCHEMATIC))

gerbers: 2layers

doc: gerbers pdf png




### GERBER FILES GENERATION

2layers: $(PROJECT_NAME).GTL $(PROJECT_NAME).GTS $(PROJECT_NAME).GTO $(PROJECT_NAME).GTP $(PROJECT_NAME).GBL $(PROJECT_NAME).GBS $(PROJECT_NAME).GBO $(PROJECT_NAME).GBP  $(PROJECT_NAME).TXT $(PROJECT_NAME).GML 
	zip $(PROJECT_NAME)-gerbers.zip $(PROJECT_NAME).{GTL,GTS,GTO,GTP,GBL,GBS,GBO,GBP,TXT,dri,gpi}
	
# Top layer - copper
%.GTL : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $@ $< Top Pads Vias
# Top layer - solder stop mask
%.GTS : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $@ $< tStop
# Top layer - silkscreen
%.GTO : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $@ $< Dimension Document
# Top layer - solder paste
%.GTP: %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $@ $< tCream

# Bottom layer - copper
%.GBL : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $@ $< Bottom Pads Vias
# Bottom layer - solder stop mask
%.GBS : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $@ $< bStop
# Bottom layer - silkscreen
%.GBO : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $@ $< Dimension
# Bottom layer - solder paste
%.GBP: %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $@ $< bCream

# Drills and holes
%.TXT : %.brd
	$(EAGLE) -X -N -d EXCELLON -E -0.01 -E 0.1 -o $@ $< Drills Holes

# Board outline
%.GML : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $@ $< Dimension

# Component placement
%.place : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $@ $< tPlace tDocu


### DOCUMENTATION GENERATION
pdf:
	$(EAGLE) -X -N -d PS -x.4 -y.2 -o $(PROJECT_NAME)-schematic.ps $(SCHEMATIC) Nets Busses Symbols Names Values
	ps2pdf $(PROJECT_NAME)-schematic.ps $(PROJECT_NAME)-schematic.pdf
	@rm $(PROJECT_NAME)-schematic.ps
	
png:  $(PROJECT_NAME).GTL $(PROJECT_NAME).GTS $(PROJECT_NAME).GTO $(PROJECT_NAME).GTP $(PROJECT_NAME).GBL $(PROJECT_NAME).GBS $(PROJECT_NAME).GBO $(PROJECT_NAME).GBP  $(PROJECT_NAME).TXT $(PROJECT_NAME).GML $(PROJECT_NAME).place
	`gerbv --export=png --output=$(PROJECT_NAME)-layout.png --dpi=$(GERBER_IMAGE_RESOLUTION) \
	--background=#000000 --foreground=#$(HOLES_COLOUR) $(PROJECT_NAME).TXT \
	--foreground=#$(SILKSCREEN_COLOUR) $(PROJECT_NAME).GTO \
	--foreground=#$(SILKSCREEN_COLOUR) $(PROJECT_NAME).place \
	--foreground=#$(PADS_COLOUR) $(PROJECT_NAME).GTS \
	--foreground=#$(TOP_SOLDERMASK_COLOUR) $(PROJECT_NAME).GTL \
	--foreground=#$(BOTTOM_SOLDERMASK_COLOUR) $(PROJECT_NAME).GBL `

### CLEAN ###

clean:
	rm -rf $(PROJECT_NAME).{GTL,GTS,GTO,GTP,GBL,GBS,GBO,GBP,TXT,GML,dri,gpi,place}
	rm -rf $(PROJECT_NAME)-gerbers.zip
	rm -rf $(PROJECT_NAME)-schematic.pdf
	rm -rf $(PROJECT_NAME)-layout.png
