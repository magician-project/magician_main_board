#!/bin/bash

#Just use the arduino ide :P
#This was for trying different avr-gcc flags

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"


#sudo apt update
#sudo apt install avr-libc gcc-avr avrdude
#Simple dependency checker that will apt-get stuff if something is missing
SYSTEM_DEPENDENCIES="build-essential avr-libc gcc-avr avrdude"

for REQUIRED_PKG in $SYSTEM_DEPENDENCIES
do
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo "Checking for $REQUIRED_PKG: $PKG_OK"
if [ "" = "$PKG_OK" ]; then

  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."

  #If this is uncommented then only packages that are missing will get prompted..
  #sudo apt-get --yes install $REQUIRED_PKG

  #if this is uncommented then if one package is missing then all missing packages are immediately installed..
  sudo apt-get update && sudo apt-get install $SYSTEM_DEPENDENCIES  
  break
fi
done
#------------------------------------------------------------------------------



set -e


CORE_BASE="$HOME/.arduino15/packages/arduino/hardware/avr/1.8.6"
#OR
#git clone https://github.com/arduino/ArduinoCore-avr
#CORE_BASE="$DIR/ArduinoCore-avr"

CORE_PATH="$CORE_BASE/cores/arduino"
WIRE_PATH="$CORE_BASE/libraries/Wire/src"
TWI_PATH="$CORE_BASE/libraries/Wire/src/utility"
SERIAL_PATH="$CORE_BASE/libraries/SoftwareSerial/src"
VARIANT_PATH="$CORE_BASE/variants/eightanaloginputs"


SKETCH_NAME="CameraControllerNano"
INO_FILE="$SKETCH_NAME.ino"
BUILD_DIR="build"
HEX_FILE="$BUILD_DIR/$SKETCH_NAME.hex"
PORT="/dev/ttyUSB0"
MCU="atmega328p"
F_CPU="16000000UL"
PROGRAMMER="arduino"
BAUD_RATE=57600  # Use 57600 for old bootloader

# Create build directory
mkdir -p $BUILD_DIR


#if [ -d $BUILD_DIR/src ]
#then
#echo "Found linked files" 
#else 
#echo "Linking files" 
#cd $BUILD_DIR
#ln -s ../src/
#cd ..
#fi

# Convert .ino to .cpp
CPP_FILE="$BUILD_DIR/$SKETCH_NAME.cpp"
echo "// Auto-generated .cpp file" > "$CPP_FILE"
echo "#include <Arduino.h>" >> "$CPP_FILE"
cat "$INO_FILE" >> "$CPP_FILE"


# Collect all source files from src/
SRC_FILES=$(find . \( -name '*.cpp' -o -name '*.c' \))
INCLUDE_PATHS=$(find . -type d | sed 's/^/-I/')
SRC_FILES="$SRC_FILES"
#SRC_FILES="$CPP_FILE"

#echo "SRC: $SRC_FILES"
#echo "INCLUDE_PATHS: $INCLUDE_PATHS"

# Compile core files (including wiring.c and others)
CORE_OBJECTS=""
CORE_SRC=""

# Compile C core files with avr-gcc
for file in "$CORE_PATH"/*.c; do
    #CORE_SRC="$file $CORE_SRC"
    obj="$BUILD_DIR/$(basename "$file").o"
    avr-gcc -c -Os -std=gnu11 -mmcu=$MCU -DF_CPU=$F_CPU -I"$CORE_PATH" -I"$VARIANT_PATH" "$file" -o "$obj"
    CORE_OBJECTS="$CORE_OBJECTS $obj"
done

for file in "$TWI_PATH"/*.c; do
    #CORE_SRC="$file $CORE_SRC"
    obj="$BUILD_DIR/$(basename "$file").o"
    avr-gcc -c -Os -std=gnu11  -mmcu=$MCU -DF_CPU=$F_CPU -I"$CORE_PATH" -I"$WIRE_PATH" -I"$VARIANT_PATH" "$file" -o "$obj"
    CORE_OBJECTS="$CORE_OBJECTS $obj"
done


#Just add CPP core files to get dynamically compiled
for file in "$WIRE_PATH"/*.cpp; do
    CORE_SRC="$file $CORE_SRC"
done

for file in "$SERIAL_PATH"/*.cpp; do
    CORE_SRC="$file $CORE_SRC"
done

for file in "$CORE_PATH"/*.cpp; do
    CORE_SRC="$file $CORE_SRC"
done

for file in "$CORE_PATH"/*.S; do
    CORE_SRC="$file $CORE_SRC"
done


# Compile all user source files from src/
echo "Compiling user code..."
USER_OBJECTS=""

for file in $SRC_FILES; do
    filename=$(basename "$file")
    obj="$BUILD_DIR/${filename}.o"
    ext="${file##*.}"

    if [[ "$ext" == "cpp" ]]; then
        avr-g++ -c -Os -std=gnu++17 -mmcu=$MCU -DF_CPU=$F_CPU \
            -I"$CORE_PATH" -I"$WIRE_PATH" -I"$VARIANT_PATH" $INCLUDE_PATHS \
            "$file" -o "$obj"
    elif [[ "$ext" == "c" ]]; then
        avr-gcc -c -Os -std=gnu11 -mmcu=$MCU -DF_CPU=$F_CPU \
            -I"$CORE_PATH" -I"$WIRE_PATH" -I"$VARIANT_PATH" $INCLUDE_PATHS \
            "$file" -o "$obj"
    fi

    USER_OBJECTS="$USER_OBJECTS $obj"
done


# Compile
echo "Compiling $BUILD_DIR/$SKETCH_NAME.elf"
avr-g++ -flto  -s -Os -std=gnu++17 -mmcu=$MCU -DF_CPU=$F_CPU  -I"$CORE_PATH" -I"$WIRE_PATH" -I"$TWI_PATH" -I"$SERIAL_PATH" -I"$VARIANT_PATH" $INCLUDE_PATHS $SRC_FILES $CORE_SRC $CORE_OBJECTS -o "$BUILD_DIR/$SKETCH_NAME.elf"


# Generate HEX file
echo "Generating HEX file"
avr-objcopy -O ihex -R .eeprom "$BUILD_DIR/$SKETCH_NAME.elf" "$HEX_FILE"

#Stopping before uploading anything potentially causing damage!
exit 0

# Upload with avrdude
echo "Uploading $HEX_FILE to $PORT..."
avrdude -v -patmega328p -carduino -P$PORT -b$BAUD_RATE -D -Uflash:w:$HEX_FILE:i

#avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b57600 -D -Uflash:w:build/$SKETCH_NAME.hex:i


echo "Upload complete!"
