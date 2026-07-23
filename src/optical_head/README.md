# PGV100 ROS2 Node (Optical_head_converter)

Questo nodo ROS2 permette di interfacciarsi con il sensore **P+F PGV100**, leggendo i dati dai tag ottici lungo il percorso e pubblicando le informazioni su specifici topic ROS2. Gestisce anche i comandi di direzione inviati da altri nodi.

---

## 1. Funzionalità principali

- Lettura dati dal PGV100 via **seriale** (tipicamente `/dev/ttyUSB0` a 115200 bps).
- Decodifica delle informazioni dai byte del pacchetto:
  - Angolo del robot
  - Posizione X e Y
  - Numero di corsie colorate
  - Presenza di corsia senza colore
  - Presenza di tag rilevato
  - ID del tag rilevato
  - Segnalazioni di errore o warning
- Pubblicazione dei dati su topic ROS2:
  - `pgv100_scan` → messaggio `optical_head::msg::PgvScanData`
  - `chatter` → messaggio `std_msgs::msg::String` (legacy)
- Ricezione dei comandi di direzione da topic ROS2:
  - `pgv_dir` → messaggio `optical_head::msg::PgvDirMsg`
- Gestione di interruzione CTRL+C chiudendo correttamente la porta seriale.

---

## 2. Topic e messaggi

### 2.1 Publisher

#### `pgv100_scan` (`optical_head::msg::PgvScanData`)

Messaggio principale con le informazioni del sensore PGV100.

| Campo | Tipo | Descrizione |
|-------|------|-------------|
| `angle` | `float32` | Angolo rilevato in gradi |
| `x_pos` | `float32` | Posizione X del robot in mm |
| `y_pos` | `float32` | Posizione Y del robot in mm |
| `direction` | `string` | Direzione selezionata (`"Right lane"`, `"Left lane"`, `"Straight Ahead"`, `"No lane is selected"`) |
| `color_lane_count` | `int32` | Numero di corsie colorate rilevate |
| `no_color_lane` | `uint8` | 1 se nessuna corsia colorata rilevata, 0 altrimenti |
| `no_pos` | `uint8` | 1 se posizione non rilevata, 0 altrimenti |
<!-- | `tag_detected` | `uint8` | 1 se tag rilevato, 0 altrimenti | -->
| `warning_string` | `string` | Messaggi di warning o errori del sensore |
| `error` | `bool` | True se bit di errore attivo, False altrimenti |
<!-- | `tag_id` | `int32` | ID del tag rilevato (da byte 13–16 del pacchetto) | -->

#### `chatter` (`std_msgs::msg::String`)

Messaggio legacy contenente solo l’**angolo** in gradi, utile per compatibilità con vecchi nodi.

---

### 2.2 Subscriber

#### `pgv_dir` (`optical_head::msg::PgvDirMsg`)

Riceve comandi di direzione da altri nodi o moduli di controllo.

| Campo | Tipo | Descrizione |
|-------|------|-------------|
| `dir_command` | `uint8` | Comando di direzione:<br>0 = nessuna corsia selezionata<br>1 = corsia destra<br>2 = corsia sinistra<br>3 = avanti |

Quando riceve un comando, il nodo invia i byte corretti al sensore PGV100 per impostare la direzione.

---

## 3. Funzionamento del nodo

1. **Avvio seriale**: apre `/dev/ttyUSB0` e configura la comunicazione seriale (115200 bps, 7 bit, 1 stop bit, senza controllo hardware).
2. **Loop principale**:
   - Invia richiesta di posizione al sensore.
   - Legge pacchetto di dati (~21 byte).
   - Decodifica:
     - `angle` dai byte 11–12
     - `x_pos` dai byte 3–6
     - `y_pos` dai byte 7–8
     - `tag_id` dai byte 13–16
     - Warning/error dai byte 19–20
   - Controlla consistenza dei dati.
   - Pubblica su `pgv100_scan` se dati validi.
3. **Gestione direzione**:
   - Riceve `dir_command` dal topic `pgv_dir`.
   - Scrive i byte corrispondenti alla direzione sul PGV100.
4. **Gestione CTRL+C**:
   - Chiude la porta seriale in modo sicuro.

---

## 4. Esempio di utilizzo

### Avviare il nodo

ros2 run optical_head optical_head_converted

## Inviare un comando di direzione

ros2 topic pub /pgv_dir optical_head/msg/PgvDirMsg "{dir_command: 3}"

