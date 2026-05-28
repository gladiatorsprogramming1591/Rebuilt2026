$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Requirements = Join-Path $ScriptDir "requirements.txt"

try {
    py -m pip install -r $Requirements
} catch {
    python -m pip install -r $Requirements
}
