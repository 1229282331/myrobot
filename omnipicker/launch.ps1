$ErrorActionPreference = 'Stop'
$taskRoot = $PSScriptRoot
$condaCommand = Get-Command conda -ErrorAction SilentlyContinue
if (-not $condaCommand) { throw 'Conda is not on PATH. Open an Anaconda PowerShell Prompt and run this script.' }
$envList = conda env list --json | ConvertFrom-Json
$studioEnv = @($envList.envs | Where-Object { (Split-Path $_ -Leaf) -eq 'omnipicker' })
if ($studioEnv.Count -ne 1) { throw 'Create the omnipicker environment first: conda env create -f environment.yml' }
$studioPython = Join-Path $studioEnv[0] 'python.exe'
& $studioPython -c 'import sys, PySide6, usb, libusb_package, numpy; assert sys.version_info[:2] == (3,11)'
if ($LASTEXITCODE -ne 0) { throw 'Environment check failed. See README.md.' }
# This is the interactive application requested by the user, so show its window.
Start-Process -FilePath (Join-Path $studioEnv[0] 'pythonw.exe') -ArgumentList '-m','studio' -WorkingDirectory $taskRoot
