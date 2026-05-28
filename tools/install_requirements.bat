@echo off
py -m pip install -r "%~dp0requirements.txt"
if errorlevel 1 (
  python -m pip install -r "%~dp0requirements.txt"
)
