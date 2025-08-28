#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

zip magician_main_board.zip magicianLogo.svg magician_robot.svg magician_main_board.kicad_sch magician_main_board.kicad_pcb magician_main_board.kicad_pro magician_main_board.kicad_prl


echo "scp -P 2222 magician_main_board.zip ammar@ammar.gr:/home/ammar/public_html/magician"

exit 0
