# Configurazione della Testa Ottica PGV via USB

Per far riconoscere correttamente la testa ottica PGV collegata via USB su Linux e ROS2, è necessario creare una **regola udev** che permetta di mappare il device USB a un percorso stabile, ad esempio `/dev/pgv100`.

## Passaggi

1. **Identificare il device USB**
```
lsusb
```

Cerca il dispositivo PGV, ad esempio:
ID 0403:6015 Future Technology Devices International.

2. **Identificare il device USB**
Crea il file /etc/udev/rules.d/99-pgv.rules con il contenuto:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="pgv100"
```
Questo crea un collegamento simbolico stabile /dev/pgv100 verso il device reale.

3. **Ricaricare le regole udev**

```
sudo udevadm control --reload-rules
sudo udevadm trigger

```
4. **Verifica**

```
ls -l /dev/pgv100
```
Dovresti vedere un link simbolico verso /dev/ttyUSB*.

5. **Aggiornare il file YAML dei sensori ROS2**

```
optical_head:
  node_name: pgv100_node
  namespace: optical_head
  frame_id: base_footprint
  serial_port: "/dev/pgv100"
```
Con questa configurazione, il nodo ROS2 della testa ottica può aprire /dev/pgv100 senza problemi di device non trovato o nomi dinamici.