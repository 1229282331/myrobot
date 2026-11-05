param([string]$Conda = 'conda')
$ErrorActionPreference = 'Stop'
$taskRoot = $PSScriptRoot
$taskPackages = @(
    @{
        Uri = 'https://github.com/orbbec/OrbbecSDK/releases/download/v1.10.37/OrbbecViewer_v1.10.37_202607071428_win_x64_release.zip'
        Archive = 'downloads\OrbbecViewer-v1.10.37-win-x64.zip'
        Target = 'vendor\orbbec-v1.10.37\viewer'
        Hash = '9B0F738B61FE6901F9F86EB6EAB8BBC53E3DE532EB61C1AED3557BE211EFAD8A'
    },
    @{
        Uri = 'https://github.com/orbbec/OrbbecSDK/releases/download/v1.10.37/OrbbecSDK_C_C%2B%2B_v1.10.37_20260707_3f75820b8_win_x64_release.zip'
        Archive = 'downloads\OrbbecSDK-v1.10.37-win-x64.zip'
        Target = 'vendor\sdk-v1.10.37'
        Hash = '22F6330C3E1C9143B9F4FDCB8BC3216DEF2451B6C35174A8511528EAF8D0CCA2'
    }
)
New-Item -ItemType Directory -Force -Path (Join-Path $taskRoot 'downloads'), (Join-Path $taskRoot 'vendor'), (Join-Path $taskRoot 'logs') | Out-Null
foreach ($taskPackage in $taskPackages) {
    $taskArchive = Join-Path $taskRoot $taskPackage.Archive
    if (-not (Test-Path -LiteralPath $taskArchive)) {
        Invoke-WebRequest -Uri $taskPackage.Uri -OutFile $taskArchive -UseBasicParsing
    }
    if ((Get-FileHash -LiteralPath $taskArchive -Algorithm SHA256).Hash -ne $taskPackage.Hash) {
        throw "Unexpected archive hash: $taskArchive"
    }
    Expand-Archive -LiteralPath $taskArchive -DestinationPath (Join-Path $taskRoot $taskPackage.Target) -Force
}
$taskRuntime = Join-Path $taskRoot '.runtime'
$taskPy = Join-Path $taskRuntime 'python.exe'
if (-not (Test-Path -LiteralPath $taskPy)) {
    & $Conda create --prefix $taskRuntime python=3.11 pip -y
    if ($LASTEXITCODE -ne 0) { throw 'Python runtime creation failed' }
}
& $taskPy -m pip install --disable-pip-version-check -r (Join-Path $taskRoot 'requirements.txt')
if ($LASTEXITCODE -ne 0) { throw 'Python dependencies failed' }
Write-Host 'Runtime ready. The Orbbec Windows depth driver must also be installed once; see README.md.'
