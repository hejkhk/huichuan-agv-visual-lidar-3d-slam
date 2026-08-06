#Requires -RunAsAdministrator

$ErrorActionPreference = "Stop"

if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
    throw "winget is unavailable. Install or update Microsoft App Installer first."
}

Write-Host "[1/2] Enabling WSL 2 platform (no Linux distribution is installed)..."
$Features = @(
    "Microsoft-Windows-Subsystem-Linux",
    "VirtualMachinePlatform"
)
foreach ($Feature in $Features) {
    $State = (Get-WindowsOptionalFeature -Online -FeatureName $Feature).State
    if ($State -ne "Enabled") {
        $Result = Enable-WindowsOptionalFeature -Online -FeatureName $Feature `
            -All -NoRestart
        if ($Result.RestartNeeded) {
            Write-Host "  $Feature enabled (restart required)"
        } else {
            Write-Host "  $Feature enabled"
        }
    } else {
        Write-Host "  $Feature already enabled"
    }
}

Write-Host "[2/2] Installing Docker Desktop..."
& winget install --exact --id Docker.DockerDesktop `
    --accept-package-agreements --accept-source-agreements
if ($LASTEXITCODE -ne 0) {
    throw "Docker Desktop installation failed with exit code $LASTEXITCODE"
}

Write-Host ""
Write-Host "Prerequisites installed. Restart Windows before running Docker Desktop."
Write-Host "After restart, start Docker Desktop and run:"
Write-Host "  .\tools\arm64_humble\BUILD_ARM64_HUMBLE.ps1 -Mode quick"
