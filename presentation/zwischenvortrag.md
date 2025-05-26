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

<!-- Simon -->
## 1. Projektstand

![Notion-Organisation](../Img/notion_zwischenstand.png "Übersicht über Aufgaben und Fristen zum Robotik Projekt in Notion-Datenbank")

---

<!-- Fabian -->
## 2. Systemarchitektur

**Überlegungen:**

+ Struktur des Projektes als einzelnes Package
+ Erweiterung durch hinzufügen von Nodes mit neuem Funktionsumfang
+ Planung einer topic-Struktur sinnvoll, um Konflikte zu vermeiden und Einheitlichkeit zu wahren
+ Planung von Publisher und Subscriber, um gewollte Funktionalität bestmöglich umzusetzen

**Topics und Nodes:**

+ Visualisierung der Node-Interaktionen (Nodes und Topics):

![Architektur](../Img/node_overview_extended.png "Übersicht der ROS2-Nodes und Datenflüsse, erstellt mit rqt_graph")

---

<!-- Lucas -->
## 3. Kernfunktionen und Demonstration

+ **Ampelerkennung:**

  ![Ampelerkennung](../Img/traffic_light.png "Debug Bilder zur Ampelerkennung")

+ **Hindernisumfahrung:**

  !?[Demo-Video Hindernisumfahrung](https://youtube.com/shorts/McEjPxxlAfo "Demo-Video zum Fahrmanövers Hidnernisumfahrung")

+ **Park-Schild-Erkennung:**

  ![Park-Schild-Erkennung](../Img/parking_sign.png "Debug Bild zur Park-Schild-Erkennung")

<!-- Simon -->
+ **Park-Manöver:**

  !?[Demo-Video Park-Manöver](https://youtube.com/shorts/R6ZFzQ-cY1E "Demo-Video zum Park-Manöver")

+ **GUI Debug Node und Keyboard Handler:**

  ![Übersicht Topics und Nodes für Debug-Canva](../Img/debug_topics_nodes.png "Übersicht der Topics und Nodes, die für das Debug-Canva genutzt werden, erstellt mit rqt_graph")
  ![Debug Canva](../Img/debug_canva.png "Aktuelles Debug Canva")
  + Keyboard Handler: Möglichkeit zum manuellen Stoppen des Roboters
  + in separatem Terminal mit den Tasten 's' für Stop/Start und 'd' zum Togglen des Debug Canva

---

<!-- Lucas -->
## 4. Ausblick

+ Spiegelung und Reflexion auf der Fahrbahn müssen noch besser behandelt werden
+ Erweitertung um Krezungsmanöver

**Vielen Dank für Ihre Aufmerksamkeit!**

**Fragen?**
