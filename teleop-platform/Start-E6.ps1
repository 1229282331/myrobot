param(
    [ValidateSet('shadow', 'live')][string]$Stage = 'shadow',
    [ValidateSet('commissioning', 'intermediate', 'normal')][string]$MotionProfile = 'commissioning',
    [ValidateSet('moveit_servo', 'native_servop')][string]$MotionEngine = 'moveit_servo',
    [ValidateSet('ros', 'cpp')][string]$MoveItApi,
    [ValidateSet('local', 'model')][string]$WorkspaceMode,
    [ValidateSet('flange', 'omnipicker-r2', 'omnipicker-r4')][string]$TcpProfile = 'omnipicker-r4',
    [ValidateSet('none', 'ruckig')][string]$NativeSmoothing = 'none',
    [ValidateSet(0.05, 0.1)][double]$ServoDuration = 0.1,
    [ValidateSet('h264', 'vp8')][string]$PreviewCodec = 'h264',
    [switch]$LatencyMarker,
    [switch]$Password,
    [switch]$ReuseLocal,
    [switch]$NoBrowser,
    [switch]$AcceptanceInputs,
    [switch]$RecoverRecordings,
    [string]$Episode
)
$ErrorActionPreference = 'Stop'
$launcherPython = Join-Path $PSScriptRoot '.venv-launcher/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $launcherPython)) {
    throw '请先运行 python teleop-platform/scripts/setup_launcher.py，创建固定依赖启动环境。'
}
$launchArgs = @('-B', '-X', 'utf8', (Join-Path $PSScriptRoot 'scripts/start_platform.py'), '--stage', $Stage, '--motion-profile', $MotionProfile, '--motion-engine', $MotionEngine, '--native-smoothing', $NativeSmoothing)
if ($MoveItApi) { $launchArgs += @('--moveit-api', $MoveItApi) }
if ($WorkspaceMode) { $launchArgs += @('--workspace-mode', $WorkspaceMode) }
$launchArgs += @('--servo-duration-s', $ServoDuration.ToString([System.Globalization.CultureInfo]::InvariantCulture), '--preview-codec', $PreviewCodec)
$launchArgs += @('--tcp-profile', $TcpProfile)
if ($LatencyMarker) { $launchArgs += '--latency-marker' }
if ($Password) { $launchArgs += '--password' }
if ($ReuseLocal) { $launchArgs += '--reuse-local' }
if ($NoBrowser) { $launchArgs += '--no-browser' }
if ($AcceptanceInputs) { $launchArgs += '--acceptance-inputs' }
if ($RecoverRecordings) { $launchArgs += '--recover-recordings' }
if ($Episode) { $launchArgs += @('--episode', $Episode) }
& $launcherPython @launchArgs
exit $LASTEXITCODE
