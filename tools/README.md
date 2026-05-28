# PathPlanner Auto Generator

This folder contains helper files for the PathPlanner auto generator script.

Expected layout:

```text
tools/
  generate_auto.py
  README.md
  requirements.txt
  install_requirements.bat
  install_requirements.ps1
  install_requirements.sh
  BattlecryField26.png        optional, used by the visualizer
  navgrid.json                optional, used by the visualizer
```

The script expects the PathPlanner project to be at:

```text
../src/main/deploy/pathplanner
```

when run from the `tools/` folder.

## Install requirements

Windows PowerShell:

```powershell
cd tools
py -m pip install -r requirements.txt
```

Or run:

```powershell
.\install_requirements.ps1
```

Windows Command Prompt:

```bat
cd tools
install_requirements.bat
```

macOS/Linux:

```bash
cd tools
python3 -m pip install -r requirements.txt
```

Or run:

```bash
chmod +x install_requirements.sh
./install_requirements.sh
```

## Tkinter note

Tkinter is part of the standard Python install on most Windows Python installs. If the GUI fails with a Tkinter import error on Linux, install it with your system package manager.

Ubuntu example:

```bash
sudo apt install python3-tk
```

## Run the GUI

From the `tools/` folder:

```bash
python generate_auto.py
```

or:

```bash
python generate_auto.py gui
```

## List autos and paths

```bash
python generate_auto.py list
```

## Generate from CLI

Example one-pass dot auto:

```bash
python generate_auto.py generate --name "Test One Pass Close" --kind one-pass-dot --first safe --first-hub --dot close
```

Example two-pass final sweep:

```bash
python generate_auto.py generate --name "Test Two Pass Final" --kind two-pass-final --first risk --first-greedy --first-hub --second FarA-CloseB --localize localize --final FarA-CloseB --final-greedy
```

Example copy auto:

```bash
python generate_auto.py generate --name "Tuned Copy Auto" --kind one-pass-final --first safe --first-hub --final FarA-CloseB --copy-paths
```

Copy mode creates copied path files and prefixes linked waypoint names with the auto name so tuning the copied auto does not move the original shared paths.

## Field visualizer files

For the field image backdrop, place this file next to `generate_auto.py`:

```text
BattlecryField26.png
```

For field scaling, place this file next to `generate_auto.py` if you want the visualizer to use it from the tools folder:

```text
navgrid.json
```

If those files are not in `tools/`, the script may fall back to the PathPlanner project folder depending on the script version.
