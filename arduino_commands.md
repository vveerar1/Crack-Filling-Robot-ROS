# Arduino-Cli Commands

- Create a configuration file
```console
$ arduino-cli config init
```
- Create a new sketch
```console
$ arduino-cli sketch new MyFirstSketch
```
- Connect the board to your PC
```console
$ arduino-cli core update-index
```

```console
$ arduino-cli board list
Port         Protocol Type              Board Name  FQBN            Core
/dev/ttyACM0 serial   Serial Port (USB) Arduino Uno arduino:avr:uno arduino:avr
```
```console
$ arduino-cli board listall avr
```
- Install the core for your board

```console
$ arduino-cli core install arduino:avr
```

- Verify installed the core properly by running:
```console
$ arduino-cli core list
```
- Compile the sketch
```console
$ arduino-cli compile --fqbn arduino:avr:uno MyFirstSketch
```
- Upload the sketch
```console
$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno MyFirstSketch
```