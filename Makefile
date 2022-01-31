MPYCROSS = mpy-cross

SRC = \
	mpu6050.py \
	constants.py

OBJ = $(SRC:.py=.mpy)

%.mpy: %.py
	$(MPYCROSS) -o $@ $<

all: $(OBJ)

.PHONY: reset clean
clean:
	rm -f $(OBJ)
