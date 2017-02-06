PORT = /dev/ttyUSB0
AMPY = ampy -p $(PORT)

SRC = \
	boot.py \
	main.py \
	mpu6050.py \
	mpuserver.py
all:

install: .lastbuild

.lastbuild: $(SRC)
	set -x; for src in $?; do $(AMPY) put $$src; done
	date > .lastbuild


.PHONY: reset clean

reset:
	$(AMPY) reset

clean:
	rm -f .lastbuild
