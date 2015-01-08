# Eagle executable path
EAGLE := /opt/eagle-7.2.0/bin/eagle

# Gerbv PCB image preview parameters - colours, plus resolution
GERBER_IMAGE_RESOLUTION?=300
BACKGROUND_COLOUR?=000000
HOLES_COLOUR?=000000
SILKSCREEN_COLOUR?=C4C4C4
PADS_COLOUR?=FFDE4E
TOP_SOLDERMASK_COLOUR?=8D2323CC
BOTTOM_SOLDERMASK_COLOUR?=23238D

# Constants
SHELL := /bin/bash
MKDIR := mkdir -p
MAKEFILE_PATH := $(abspath $(MAKEFILE_LIST))
PROJECT_NAME := $(notdir $(patsubst %/hw/,%,$(dir $(MAKEFILE_PATH))))
BOARD := $(PROJECT_NAME).brd
SCHEMATIC := $(PROJECT_NAME).sch
GERBERS_PATH := ./gerbers

all: gerbers doc

debug:
	$(info MAKEFILE_PATH: $(MAKEFILE_PATH))
	$(info PROJECT_NAME: $(PROJECT_NAME))
	$(info BOARD: $(BOARD))
	$(info SCHEMATIC: $(SCHEMATIC))

gerbers: mkdir 2layers

doc: gerbers pdf png




### GERBER FILES GENERATION

mkdir:
	$(MKDIR) $(GERBERS_PATH)

2layers: $(PROJECT_NAME).GTL $(PROJECT_NAME).GTS $(PROJECT_NAME).GTO $(PROJECT_NAME).GTP $(PROJECT_NAME).GBL $(PROJECT_NAME).GBS $(PROJECT_NAME).GBO $(PROJECT_NAME).GBP  $(PROJECT_NAME).TXT $(PROJECT_NAME).GML $(PROJECT_NAME).place 
	zip $(PROJECT_NAME)-gerbers.zip $(GERBERS_PATH)/*
	
# Top layer - copper
%.GTL : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $(GERBERS_PATH)/$@ $< Top Pads Vias
# Top layer - solder stop mask
%.GTS : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $(GERBERS_PATH)/$@ $< tStop
# Top layer - silkscreen
%.GTO : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $(GERBERS_PATH)/$@ $< Dimension Document
# Top layer - solder paste
%.GTP: %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $(GERBERS_PATH)/$@ $< tCream

# Bottom layer - copper
%.GBL : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $(GERBERS_PATH)/$@ $< Bottom Pads Vias
# Bottom layer - solder stop mask
%.GBS : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $(GERBERS_PATH)/$@ $< bStop
# Bottom layer - silkscreen
%.GBO : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $(GERBERS_PATH)/$@ $< Dimension
# Bottom layer - solder paste
%.GBP: %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $(GERBERS_PATH)/$@ $< bCream

# Drills and holes
%.TXT : %.brd
	$(EAGLE) -X -N -d EXCELLON -E -0.01 -E 0.1 -o $(GERBERS_PATH)/$@ $< Drills Holes

# Board outline
%.GML : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $(GERBERS_PATH)/$@ $< Dimension

# Component placement
%.place : %.brd
	$(EAGLE) -X -N -d GERBER_RS274X -o $(GERBERS_PATH)/$@ $< tPlace tDocu


### DOCUMENTATION GENERATION
pdf:
	$(EAGLE) -X -N -d PS -x.4 -y.2 -o $(PROJECT_NAME)-schematic.ps $(SCHEMATIC) Nets Busses Symbols Names Values
	ps2pdf $(PROJECT_NAME)-schematic.ps $(PROJECT_NAME)-schematic.pdf
#	@rm $(PROJECT_NAME)-schematic.ps
	
png:  $(PROJECT_NAME).GTL $(PROJECT_NAME).GTS $(PROJECT_NAME).GTO $(PROJECT_NAME).GTP $(PROJECT_NAME).GBL $(PROJECT_NAME).GBS $(PROJECT_NAME).GBO $(PROJECT_NAME).GBP  $(PROJECT_NAME).TXT $(PROJECT_NAME).GML $(PROJECT_NAME).place
	`gerbv --export=png --output=$(PROJECT_NAME)-layout.png --dpi=$(GERBER_IMAGE_RESOLUTION) \
	--background=#000000 --foreground=#$(HOLES_COLOUR) $(GERBERS_PATH)/$(PROJECT_NAME).TXT \
	--foreground=#$(SILKSCREEN_COLOUR) $(GERBERS_PATH)/$(PROJECT_NAME).GTO \
	--foreground=#$(SILKSCREEN_COLOUR) $(GERBERS_PATH)/$(PROJECT_NAME).place \
	--foreground=#$(PADS_COLOUR) $(GERBERS_PATH)/$(PROJECT_NAME).GTS \
	--foreground=#$(TOP_SOLDERMASK_COLOUR) $(GERBERS_PATH)/$(PROJECT_NAME).GTL \
	--foreground=#$(BOTTOM_SOLDERMASK_COLOUR) $(GERBERS_PATH)/$(PROJECT_NAME).GBL `

### CLEAN ###

clean:
#	rm -rf $(GERBERS_PATH)/$(PROJECT_NAME).{GTL,GTS,GTO,GTP,GBL,GBS,GBO,GBP,TXT,GML,dri,gpi,place}
	rm -rf ./gerbers
	rm -rf $(PROJECT_NAME)-gerbers.zip
	rm -rf $(PROJECT_NAME)-schematic.pdf
	rm -rf $(PROJECT_NAME)-layout.png
