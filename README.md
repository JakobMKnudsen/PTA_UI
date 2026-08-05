# PTA Monitor

Desktop monitor for 24-channel PTA pressure transducer arrays over RS485.

## Capture Modes

The app has three capture modes controlled by the Log Mode selector.

### Stream

- Click Take Data once to start logging cycles continuously.
- Click Take Data again to stop logging.

### Sample

- Set Sample Count to N.
- Click Take Data once.
- The app captures exactly N cycles, then stops automatically.

### Snapshot

- Click Take Data once.
- The app captures one single measurement snapshot.

## Save Data

- Click Save Data to export captured values to CSV.
- Export format is one row per captured cycle.
- First column is timestamp.
- Remaining columns are P01..P24 values in the currently selected UI unit.

## Clear Data

- Click Clear Data to discard currently captured data without saving.
- Use this before a new run when you do not want to keep previous captures.

## Running Locally

1. Create and activate a virtual environment.
2. Install dependencies from requirements.txt.
3. Run pta_monitor.py.
