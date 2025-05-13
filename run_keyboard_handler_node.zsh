#!/bin/zsh

# Skript zum Bauen des 'waymo'-Pakets und Starten des keyboard_handler_node.
# Dieses Skript kann von jedem Verzeichnis aus ausgeführt werden.
# Es wird davon ausgegangen, dass die Conda-Umgebung ('ros2') bereits aktiviert ist.

# Definiere den Pfad zum ROS2-Workspace
ROS2_WORKSPACE_DIR="$HOME/ros2_ws"

# --- Überprüfungen und Vorbereitungen ---

# Überprüfe, ob das Workspace-Verzeichnis existiert
if [ ! -d "$ROS2_WORKSPACE_DIR" ]; then
  echo "FEHLER: ROS2 Workspace-Verzeichnis nicht gefunden unter: $ROS2_WORKSPACE_DIR"
  echo "FEHLER: Bitte stelle sicher, dass der Pfad korrekt ist und der Workspace existiert."
  exit 1 # Beendet das Skript mit einem Fehlercode
fi

# echo "INFO: ROS2 Workspace ist: $ROS2_WORKSPACE_DIR"

# --- Build-Prozess ---
# echo "INFO: Wechsle in das Workspace-Verzeichnis zum Bauen: $ROS2_WORKSPACE_DIR"
# Speichere das aktuelle Verzeichnis, um später zurückzukehren
ORIGINAL_DIR=$(pwd)
cd "$ROS2_WORKSPACE_DIR"

if [ $? -ne 0 ]; then
  echo "FEHLER: Wechsel in das Verzeichnis $ROS2_WORKSPACE_DIR fehlgeschlagen."
  exit 1
fi

# echo "INFO: Baue Paket 'waymo' im aktuellen Verzeichnis (jetzt $ROS2_WORKSPACE_DIR)..."
# Führe colcon build im Workspace-Verzeichnis aus
colcon build --packages-select waymo

# Speichere den Exit-Code von colcon
BUILD_EXIT_CODE=$?

# echo "INFO: Wechsle zurück zum ursprünglichen Verzeichnis: $ORIGINAL_DIR"
cd "$ORIGINAL_DIR"

# Überprüfe den Exit-Code des Build-Prozesses
if [ $BUILD_EXIT_CODE -ne 0 ]; then
  echo "FEHLER: colcon build des Pakets 'waymo' ist fehlgeschlagen (Exit-Code: $BUILD_EXIT_CODE)."
  exit 1
fi

# echo "INFO: Paket 'waymo' erfolgreich gebaut."

# --- Sourcing der Workspace Setup-Datei ---
# Die Setup-Dateien verwenden absolute Pfade, daher ist das aktuelle Verzeichnis hierfür weniger kritisch,
# aber der Pfad zum Workspace ($ROS2_WORKSPACE_DIR) muss korrekt sein.
SETUP_FILE_ZSH="$ROS2_WORKSPACE_DIR/install/setup.zsh"
SETUP_FILE_BASH="$ROS2_WORKSPACE_DIR/install/setup.bash"

if [ -f "$SETUP_FILE_ZSH" ]; then
  # echo "INFO: Source Workspace Setup-Datei: $SETUP_FILE_ZSH"
  source "$SETUP_FILE_ZSH"
elif [ -f "$SETUP_FILE_BASH" ]; then
  # echo "INFO: Source Workspace Setup-Datei (Fallback): $SETUP_FILE_BASH"
  source "$SETUP_FILE_BASH"
else
  echo "FEHLER: Keine passende Setup-Datei (setup.zsh oder setup.bash) im Verzeichnis $ROS2_WORKSPACE_DIR/install gefunden!"
  echo "FEHLER: Stelle sicher, dass colcon build erfolgreich war."
  exit 1
fi

# echo "INFO: Workspace-Umgebung wurde erfolgreich geladen."

# --- Starten des ROS2-Nodes ---
# echo "INFO: Starte keyboard_handler_node aus dem Paket 'waymo'..."
echo "-----------------------------------"
ros2 run waymo keyboard_handler_node

# $? enthält den Exit-Code des zuletzt ausgeführten Befehls (ros2 run)
if [ $? -ne 0 ]; then
  echo "FEHLER: Der Start von 'keyboard_handler_node' ist fehlgeschlagen oder der Node wurde unerwartet beendet."
  # Hier könntest du ggf. einen spezifischen Fehlercode zurückgeben
  # exit 2 # Beispiel für einen anderen Fehlercode als Build-Fehler
fi

# echo "INFO: keyboard_handler_node wurde beendet."