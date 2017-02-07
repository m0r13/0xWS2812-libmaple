
build:
	platformio run

upload: build
	./maple_upload ttyACM0 1 1EAF:0003 .pioenvs/maple_mini_origin/firmware.bin

serial:
	platformio device monitor
