[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://github.com/Bigfire3/waymo/blob/documentation/expose.md)

# Vorstellung: ROS2 Projekt "waymo"

## Exposé-Vortrag

<!-- data-type="none" -->
| Parameter            |Kursinformationen                                                     |
| -------------------- | -------------------------------------------------------------------- |
| **Veranstaltung:**   | `Robotik Projekt`                                                    |
| **Semester**         | `Sommersemester 2025`                                                |
| **Hochschule:**      | `Technische Universität Berkakademie Freiberg`                       |
| **Inhalte:**         | `Exposé Vortrag`                                                     |
| **Link auf GitHub:** | <https://github.com/Bigfire3/waymo/blob/documentation/expose.md>       |
| **Autoren**          | Fabian Zänker, Lucas Adler, Simon Hörtzsch @author                   |

- Gruppenmitglieder: Fabian Zänker, Lucas Adler, Simon Hörtzsch  
- Studiengang: Robotik | Mathematik in Wirtschaft, Engineering und Informatik | Angwandte Informatik
- Betreuer: Prof. Dr. Sebastian Zug, Gero Licht  
- Datum: 23.04.2025

---

## 1. Einleitung & Motivation

Entwicklung eines ros2-Packages, welches den Roboter auf Grundlage von bestimmten Fahrbahnszenarios steuert.

### Hauptziel

Roboter fährt voll automatisiert auf einer vorher nicht bekannten Fahrahn, lediglich die Fahrbahnszenarios sind durch definierte Aufgaben bekannt.

---

## 3. Projekt Organisation

![Notion-Organisation](/Img/notion.png "Übersicht über Aufgaben und Fristen zum Robotik Projekt in Notion-Datenbank")

## 2. Systemarchitektur & Komponenten

- Nodes:
  - gui_debug_node.py
  - state_manager_node.py
  - lane_detection_node.py
  - obstacle_detection_node.py
- Visualisierung der Node-Interaktionen (Nodes und Topics):

    ![Architektur](/Img/node_overview.png "Übersicht der ROS2-Nodes und Datenflüsse")

- Wichtige externe Bibliotheken:
  - OpenCV (für Auswertung und Debugging mit Kamerabildern)
  -

---

## 3. Kernfunktionen & Implementierungsdetails

- **Beispiel Funktion 1: Objekterkennung**
  - Welcher Ansatz wird verfolgt (z.B. YOLO, PointPillars)?
    - Wie wurden die Daten verarbeitet (Input/Output der Node)?
    - Besondere Herausforderungen (z.B. Echtzeitfähigkeit, Genauigkeit).
- **Beispiel Funktion 2: Pfadplanung**
  - Verwendeter Algorithmus (z.B. A*, DWA).
  - Integration mit dem Rest des Systems.

---

## 4. Demonstration & Ergebnisse

Zeige hier die Funktionsweise und Resultate.

- Link zu einem Video oder Einbettung (falls von LiaScript unterstützt).
- Screenshots aus Simulationen oder Visualisierungen (z.B. RViz2, Gazebo).
- Metriken oder qualitative Bewertung der Ergebnisse.
- Gegenüberstellung: Erwartetes vs. erreichtes Verhalten.

[Demo-Video auf YouTube](https://...)

---

## 5. Zusammenfassung & Ausblick

Fasse zusammen und blicke nach vorne.

- Wichtigste Erkenntnisse und erreichte Meilensteine des Projekts.
- Limitationen und Herausforderungen.
- Mögliche nächste Schritte:
  - Verbesserungen (z.B. Performance, Robustheit).
    - Erweiterungen (z.B. neue Sensoren, komplexere Szenarien).
    - Integration mit anderer Software/Hardware.

**Vielen Dank für Ihre Aufmerksamkeit!**

**Fragen?**
