#!/bin/bash

# File di output
# OUTPUT="dependencies.txt"
# WS_PATH=~/mini_morosai_ws  # Modifica se serve

# File di output
# Identifica il path assoluto dello script e del workspace (parent dir)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_PATH="$(dirname "$SCRIPT_DIR")"
OUTPUT="$WS_PATH/dependencies.txt"

echo "Detected Workspace Path: $WS_PATH"

echo "===== ROS2 Packages in workspace =====" > $OUTPUT
# Lista dei pacchetti nel workspace
ros2 pkg list --packages-select $(find $WS_PATH/src -maxdepth 1 -mindepth 1 -type d -exec basename {} \;) >> $OUTPUT 2>/dev/null

echo -e "\n===== ROS2 System Dependencies via rosdep =====" >> $OUTPUT
# Lista dipendenze di sistema mancanti
rosdep check --from-paths $WS_PATH/src --ignore-src >> $OUTPUT 2>&1

echo -e "\n===== Python Packages Installed =====" >> $OUTPUT
# Lista tutte librerie python3 installate
python3 -m pip freeze >> $OUTPUT

echo -e "\n===== Python Paths =====" >> $OUTPUT
# Lista path di ricerca python
python3 -c "import sys; print('\n'.join(sys.path))" >> $OUTPUT

echo "File '$OUTPUT' creato! Contiene tutte le librerie ROS2, Python e le dipendenze di sistema."

