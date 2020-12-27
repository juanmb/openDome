# Uncomment lines below if you have problems with $PATH
#SHELL := /bin/bash
#PATH := /usr/local/bin:$(PATH)

all: compile_commands.json
	pio -f -c vim run

compile_commands.json:
	pio run -t compiledb
	cp .pio/build/master-vst/compile_commands.json ./

upload:
	pio -f -c vim run --target upload

clean:
	pio -f -c vim run --target clean
	rm -f compile_commands.json

program:
	pio -f -c vim run --target program

uploadfs:
	pio -f -c vim run --target uploadfs

update:
	pio -f -c vim update

check: compile_commands.json
	clang-check src/master/main.cpp
