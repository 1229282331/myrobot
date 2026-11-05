$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    if (-not (Get-Command uv -ErrorAction SilentlyContinue)) { throw 'Install uv first: https://docs.astral.sh/uv/getting-started/installation/' }
    if (-not (Get-Command npm -ErrorAction SilentlyContinue)) { throw 'Install Node.js LTS first: https://nodejs.org/' }
    uv python install 3.12
    if ($LASTEXITCODE -ne 0) { throw 'Python installation failed' }
    $teleopPython = (uv python find 3.12 --managed-python).Trim()
    # Frozen CPU lock avoids upstream CUDA and cross-platform ARM source conflicts.
    uv sync --frozen --extra dataset --python $teleopPython
    if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed' }
    Push-Location web
    try {
        npm ci --no-audit --no-fund
        if ($LASTEXITCODE -ne 0) { throw 'npm ci failed' }
        npm run build
        if ($LASTEXITCODE -ne 0) { throw 'Frontend build failed' }
    } finally { Pop-Location }
    Write-Host 'Ready. Run teleop/launch.cmd and open http://127.0.0.1:8765'
} finally { Pop-Location }
