param(
    [string]$TunnelName = "althold-tuning",
    [string]$PublicBaseUrl = "https://altitude-ekf-tuning.flying-agents.com",
    [string]$TuningPort = "8790",
    [string]$AppServerUrl = "ws://127.0.0.1:4500",
    [string]$PiBaseUrl = "http://omrijsharon.local:8080",
    [string]$SessionSecret = $env:ALTHOLD_TUNING_SESSION_SECRET,
    [string]$OperatorSecret = $env:ALTHOLD_TUNING_OPERATOR_SECRET,
    [switch]$ForceRestart
)

$ErrorActionPreference = "Stop"

if (-not $SessionSecret) {
    throw "SessionSecret or ALTHOLD_TUNING_SESSION_SECRET is required"
}
if (-not $OperatorSecret) {
    throw "OperatorSecret or ALTHOLD_TUNING_OPERATOR_SECRET is required"
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$appDir = Join-Path $repoRoot "althold\tuning_app"
$cloudflared = (Get-Command cloudflared -ErrorAction SilentlyContinue).Source
if (-not $cloudflared) {
    $candidate = "C:\Program Files (x86)\cloudflared\cloudflared.exe"
    if (Test-Path $candidate) {
        $cloudflared = $candidate
    }
}
if (-not $cloudflared) {
    throw "cloudflared.exe was not found"
}

$codex = Get-ChildItem "$env:USERPROFILE\.vscode\extensions\openai.chatgpt-*-win32-x64\bin\windows-x86_64\codex.exe" -ErrorAction SilentlyContinue |
    Sort-Object FullName -Descending |
    Select-Object -First 1 -ExpandProperty FullName
if (-not $codex) {
    throw "codex.exe was not found under the VS Code OpenAI extension directory"
}

function Get-ListeningProcessId([int]$Port) {
    $connection = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($connection) {
        return $connection.OwningProcess
    }
    return $null
}

function Stop-ListeningProcess([int]$Port) {
    $pid = Get-ListeningProcessId $Port
    if ($pid) {
        Stop-Process -Id $pid -Force
    }
}

function Start-HiddenPowerShell([string]$Command, [string]$WorkingDirectory) {
    $bytes = [System.Text.Encoding]::Unicode.GetBytes($Command)
    $encoded = [Convert]::ToBase64String($bytes)
    Start-Process powershell.exe -ArgumentList @("-NoExit", "-ExecutionPolicy", "Bypass", "-EncodedCommand", $encoded) -WorkingDirectory $WorkingDirectory -WindowStyle Hidden
}

if ($ForceRestart) {
    Stop-ListeningProcess 4500
    Stop-ListeningProcess ([int]$TuningPort)
}

if (-not (Get-ListeningProcessId 4500)) {
    Start-HiddenPowerShell "`"$codex`" app-server --listen $AppServerUrl" $repoRoot
    Start-Sleep -Seconds 2
}

if (-not (Get-ListeningProcessId ([int]$TuningPort))) {
    $backendCommand = @"
`$env:ALTHOLD_TUNING_HOST='127.0.0.1'
`$env:ALTHOLD_TUNING_PORT='$TuningPort'
`$env:ALTHOLD_TUNING_BASE_URL='$PublicBaseUrl'
`$env:ALTHOLD_CODEX_APP_SERVER_URL='$AppServerUrl'
`$env:ALTHOLD_PI_BASE_URL='$PiBaseUrl'
`$env:ALTHOLD_TUNING_SESSION_SECRET='$SessionSecret'
`$env:ALTHOLD_TUNING_OPERATOR_SECRET='$OperatorSecret'
`$env:ALTHOLD_REPO_ROOT='$repoRoot'
npm.cmd --prefix '$appDir' run start
"@
    Start-HiddenPowerShell $backendCommand $repoRoot
    Start-Sleep -Seconds 2
}

$tunnelCommand = "`"$cloudflared`" tunnel run --url http://localhost:$TuningPort $TunnelName"
Start-HiddenPowerShell $tunnelCommand $repoRoot

Write-Host "Altitude tuning stack requested."
Write-Host "Operator URL: $PublicBaseUrl/operator"
Write-Host "Local URL: http://127.0.0.1:$TuningPort/operator"
