PORT = /dev/ttyUSB0
AMPY = ampy -p $(PORT)
MPYCROSS = mpy-cross

SRC = \
	mpu6050.py \
	constants.py \
	batmon.py \
	mpuserver.py

OBJ = boot.py $(SRC:.py=.mpy)

%.mpy: %.py
	$(MPYCROSS) -o $@ $<

all: $(OBJ)

install: .lastbuild

.lastbuild: $(OBJ)
	set -x; for src in $?; do $(AMPY) put $$src; done
	date > .lastbuild


.PHONY: reset clean

reset:
	$(AMPY) reset

clean:
	rm -f .lastbuild
