# Romulus MIDI Looper

An LV2 MIDI looper plugin that records MIDI events in loops aligned to the host transport bars.

## Features

- **Bar-aligned recording**: Recording starts and loops at bar boundaries
- **Configurable loop length**: Set loop length from 1 to 32 bars
- **Smart note handling**: Waits for note-offs of active notes at loop end
- **Re-recording**: Discard current recording and restart by toggling record enable during recording

## Controls

- **Record Enable** (toggle): Toggle from 1 to 0 to arm recording. Recording starts at the next bar.
- **Loop Length (bars)** (1-32): Sets the length of the recording loop in bars.

## Building

### Prerequisites

- GCC compiler
- LV2 development headers
- GNU Make

### Build Instructions

```bash
cd romulus_midi_looper.lv2
make
```

### Installation

To install the plugin to your user LV2 directory:

```bash
make install
```

To install system-wide (requires root):

```bash
sudo make install INSTALL_DIR=/usr/lib/lv2
```

### Uninstall

```bash
make uninstall
```

## Usage

1. Load the plugin in your LV2 host (Ardour, Carla, Qtractor, etc.)
2. Set the desired loop length in bars
3. Toggle "Record Enable" from 1 to 0 to arm recording
4. Recording will start automatically at the next bar
5. Play your MIDI parts - they will be recorded into the loop
6. Recording stops automatically when the loop length is reached
7. The loop plays back immediately, aligned to bars
8. To re-record, toggle "Record Enable" again during playback or recording

## Technical Details

- The plugin tracks active notes (note-ons without corresponding note-offs)
- At the end of recording, it waits for note-offs of any notes that were still playing
- If a note-on for the same note arrives while waiting, the wait is cancelled
- Existing note-offs in the loop are removed when waiting for new ones
- Maximum 8192 MIDI events per loop
- Hard real-time capable

## License

ISC License

## Author

Laurent Bovet
