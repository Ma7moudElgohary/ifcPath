$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
if (Test-Path (Join-Path $ScriptDir "dist")) {
    # Release ZIP layout: installer and dist/ sit next to each other.
    $Root = $ScriptDir
} elseif (Test-Path (Join-Path (Split-Path -Parent $ScriptDir) "dist")) {
    # Source checkout layout: installer lives under scripts/.
    $Root = Split-Path -Parent $ScriptDir
} else {
    throw "Could not locate the IfcPath dist/ directory next to this installer or one directory above it."
}
Set-Location $Root

if (-not (Get-Command py -ErrorAction SilentlyContinue) -and -not (Get-Command python -ErrorAction SilentlyContinue)) {
    throw "Python 3.10+ is required. Install Python, then run this script again."
}

$Launcher = if (Get-Command py -ErrorAction SilentlyContinue) { "py" } else { "python" }
if (-not (Test-Path ".venv")) {
    & $Launcher -m venv .venv
}

$Python = Join-Path $Root ".venv\Scripts\python.exe"
& $Python -m pip install --upgrade pip

$Wheel = Get-ChildItem (Join-Path $Root "dist\ifcpath-*.whl") | Select-Object -First 1
if (-not $Wheel) {
    throw "IfcPath wheel was not found in dist/. Use the GitHub local-app release bundle."
}

& $Python -m pip install $Wheel.FullName
& $Python -m pip install "fastapi>=0.115,<1" "uvicorn>=0.34,<1" "jupedsim==1.4.2"

Write-Host "Starting IfcPath..."
& (Join-Path $Root ".venv\Scripts\ifcpath-app.exe")
