#!/bin/zsh

# Dieses Skript baut und startet das 'waymo' Paket in deinem ROS 2 Workspace.
# WICHTIG: Aktiviere deine Conda ROS-Umgebung, BEVOR du dieses Skript ausführst!
# Beispiel: conda activate ros2

# --- Konfiguration ---
# Passe diesen Pfad an, falls dein Workspace woanders liegt oder anders heißt!
WORKSPACE_DIR="$HOME/ROS2_WS"
PACKAGE_TO_BUILD="waymo"
LAUNCH_PACKAGE="waymo"
LAUNCH_FILE="waymo_launch.py"
# ---------------------

# Prüfen, ob die Conda-Umgebung aktiv zu sein scheint (einfacher Check)
if [ -z "$CONDA_PREFIX" ] || [[ "$CONDA_DEFAULT_ENV" != *"ros2"* ]]; then
  echo "FEHLER: Conda-Umgebung 'ros2' scheint nicht aktiv zu sein."
  echo "Bitte zuerst 'conda activate ros2' ausführen."
  exit 1
fi

# In den Workspace wechseln
echo "Wechsle zu $WORKSPACE_DIR"
cd "$WORKSPACE_DIR" || { echo "Fehler: Workspace-Verzeichnis '$WORKSPACE_DIR' nicht gefunden."; exit 1; }

# Spezifisches Paket bauen
echo "Baue Paket: $PACKAGE_TO_BUILD ..."
colcon build --packages-select "$PACKAGE_TO_BUILD"
BUILD_EXIT_CODE=$? # Exit-Code des Build-Prozesses speichern

# Prüfen, ob der Build erfolgreich war
if [ $BUILD_EXIT_CODE -ne 0 ]; then
  echo "Fehler: colcon build fehlgeschlagen mit Exit-Code $BUILD_EXIT_CODE."
  exit $BUILD_EXIT_CODE
fi
echo "Build erfolgreich."

# Workspace Setup-Datei sourcen (Zsh-Version)
# Die Haupt-ROS-Umgebung ist bereits durch Conda aktiv!
WORKSPACE_SETUP_FILE="$WORKSPACE_DIR/install/local_setup.zsh"
echo "Suche Workspace-Setup: $WORKSPACE_SETUP_FILE"
if [ -f "$WORKSPACE_SETUP_FILE" ]; then
  echo "Source Workspace-Setup..."
  source "$WORKSPACE_SETUP_FILE"
else
  # Fallback auf .bash, falls .zsh nicht existiert
  WORKSPACE_SETUP_FILE_BASH="$WORKSPACE_DIR/install/local_setup.bash"
  if [ -f "$WORKSPACE_SETUP_FILE_BASH" ]; then
      echo "Warnung: local_setup.zsh nicht gefunden, versuche local_setup.bash..."
      source "$WORKSPACE_SETUP_FILE_BASH"
  else
      echo "Fehler: Weder local_setup.zsh noch local_setup.bash im install-Ordner gefunden."
      exit 1
  fi
fi

# ROS 2 Launch-Datei ausführen
echo "Starte Launch-Datei: $LAUNCH_PACKAGE $LAUNCH_FILE ..."
ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE"

LAUNCH_EXIT_CODE=$?
echo "Launch-Prozess beendet mit Exit-Code $LAUNCH_EXIT_CODE."

exit $LAUNCH_EXIT_CODE