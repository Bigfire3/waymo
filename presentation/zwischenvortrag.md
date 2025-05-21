[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://github.com/Bigfire3/waymo/blob/documentation/presentation/zwischenvortrag.md)

# Zwischenvortrag: ROS 2 Projekt "waymo"

<!-- data-type="none" -->
| Parameter            | Kursinformationen                                                                     |
| -------------------- | --------------------------------------------------------------------------------------|
| **Veranstaltung:**   | `Robotik Projekt`                                                                     |
| **Semester**         | `Sommersemester 2025`                                                                 |
| **Hochschule:**      | `Technische Universität Berkakademie Freiberg`                                        |
| **Inhalte:**         | `Exposé Vortrag`                                                                      |
| **Link auf GitHub:** | https://github.com/Bigfire3/waymo/blob/documentation/presentation/zwischenvortrag.md  |
| **Autoren**          | Fabian Zänker, Lucas Adler, Simon Hörtzsch @author                                    |

+ Gruppenmitglieder: Fabian Zänker, Lucas Adler, Simon Hörtzsch  
+ Studiengang: Robotik | Mathematik in Wirtschaft, Engineering und Informatik | Angewandte Informatik
+ Betreuer: Prof. Dr. Sebastian Zug, Gero Licht  
+ Datum: 28.05.2025

---

## 1. Aktueller Projektstand

![Notion-Organisation](../Img/notion_zwischenstand.png "Übersicht über Aufgaben und Fristen zum Robotik Projekt in Notion-Datenbank")

---

## 2. Planung der Nodes und Topics

**Überlegungen:**

+ Struktur des Projektes als einzelnes Package
+ Erweiterung durch hinzufügen von Nodes mit neuem Funktionsumfang
+ Planung einer topic-Struktur sinnvoll, um Konflikte zu vermeiden und Einheitlichkeit zu wahren
+ Planung von Publisher und Subscriber, um gewollte Funktionalität bestmöglich umzusetzen

**Topic-Struktur:**

![Topic-Struktur](../Img/topic_structure.png "Übersicht über Subs/Pubs, Topics und Nodes")

## 3. Systemarchitektur & Komponenten

+ Nodes:
  
  + `gui_debug_node.py`
  + `state_manager_node.py`
  + `lane_detection_node.py`
  + `obstacle_detection_node.py`
  + `traffic_light_detection_node.py`
  + `passing_obstacle_node.py`
  + `sign_detection_node.py`
  + `parking_node.py`

+ Visualisierung der Node-Interaktionen (Nodes und Topics):

    ![Architektur](../Img/node_overview_extended.png "Übersicht der ROS2-Nodes und Datenflüsse, erstellt mit rqt_graph")

+ Wichtige externe Bibliotheken:

  + OpenCV (für Auswertung und Debugging mit Kamerabildern)
  + NumPy
  + SciPy
  + Matplotlib

---

## 4. Kernfunktionen & Implementierungsdetails

+ **Ampelerkennung:**

  + ...

+ **Hindernisumfahrung:**

  + ...

+ **Park-Schild-Erkennung:**

  + ...

+ **Park-Manöver:**

  + ...

+ **Rolle des State-Managers:**

  + ...

+ **GUI Debug Node und Keyboard Handler:**

  + ...

---

## 5. Demonstration & Ergebnisse

+ ...

---

## 6. Ausblick

+ Spiegelung und Reflexion auf der Fahrbahn müssen noch besser behandelt werden

**Vielen Dank für Ihre Aufmerksamkeit!**

**Fragen?**
