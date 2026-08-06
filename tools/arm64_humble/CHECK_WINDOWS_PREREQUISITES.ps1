$ErrorActionPreference = "Stop"

$Features = @(
    "Microsoft-Windows-Subsystem-Linux",
    "VirtualMachinePlatform"
)

Write-Host "Windows ARM64/Humble build prerequisites"
Write-Host "=========================================="

foreach ($Feature in $Features) {
    $State = (Get-WindowsOptionalFeature -Online -FeatureName $Feature).State
    Write-Host ("{0,-38} {1}" -f $Feature, $State)
}

$Docker = Get-Command docker -ErrorAction SilentlyContinue
if (-not $Docker) {
    $DockerBin = "C:\Program Files\Docker\Docker\resources\bin"
    if (Test-Path (Join-Path $DockerBin "docker.exe")) {
        $env:Path += ";$DockerBin"
        $Docker = Get-Command docker -ErrorAction SilentlyContinue
    }
}
Write-Host ("{0,-38} {1}" -f "Docker CLI", $(if ($Docker) { $Docker.Source } else { "MISSING" }))

try {
    $WslStatus = (& wsl --status 2>&1 | Out-String).Trim()
} catch {
    $WslStatus = "MISSING"
}
Write-Host ("{0,-38} {1}" -f "WSL status", $WslStatus)

if (-not $Docker) {
    Write-Host ""
    Write-Host "Required setup from an Administrator PowerShell:"
    Write-Host "  .\tools\arm64_humble\INSTALL_WINDOWS_PREREQUISITES.ps1"
    Write-Host "  Restart Windows, then start Docker Desktop"
    Write-Host "  Start Docker Desktop and enable the WSL 2 engine"
    exit 2
}

try {
    docker version | Out-Null
} catch {
    throw "Docker Desktop is installed but its Linux engine is not running."
}
docker buildx version
Write-Host "[PASS] Docker Buildx is available."
