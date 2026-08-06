param(
    [ValidateSet("quick", "full")]
    [string]$Mode = "quick",
    [switch]$NoCache
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$Dockerfile = Join-Path $PSScriptRoot "Dockerfile"

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    throw "Docker is not installed. Run CHECK_WINDOWS_PREREQUISITES.ps1 first."
}

docker version | Out-Null
docker buildx version | Out-Null

$FullVendor = if ($Mode -eq "full") { "1" } else { "0" }
$Tag = "huichuan-agv-humble-arm64:$Mode"
$Arguments = @(
    "buildx", "build",
    "--platform", "linux/arm64",
    "--build-arg", "FULL_VENDOR=$FullVendor",
    "--progress", "plain",
    "--load",
    "--tag", $Tag,
    "--file", $Dockerfile
)
if ($NoCache) {
    $Arguments += "--no-cache"
}
$Arguments += $Root

Write-Host "[ARM64] Mode      : $Mode"
Write-Host "[ARM64] Target    : Ubuntu 22.04 / ROS 2 Humble / linux-arm64"
Write-Host "[ARM64] Image tag : $Tag"

& docker @Arguments
if ($LASTEXITCODE -ne 0) {
    throw "ARM64 Humble build failed with exit code $LASTEXITCODE"
}

docker run --rm --platform linux/arm64 $Tag bash -lc `
    'test "$(uname -m)" = aarch64 && echo "[PASS] aarch64 Humble image compiled successfully"'
if ($LASTEXITCODE -ne 0) {
    throw "ARM64 image architecture verification failed"
}
