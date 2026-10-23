# ==============================================================================
# Adaptive Net: Windows PowerShell Simulation Runner & Plotter (Audit 8)
# Launches NS-3 multi-seed batch runs inside WSL2 and generates plots
# ==============================================================================
$ErrorActionPreference = "Stop"

Write-Host "==================================================================" -ForegroundColor Cyan
Write-Host "     Adaptive Net: Multi-Seed Experiment Suite & Visualizer       " -ForegroundColor Cyan
Write-Host "==================================================================" -ForegroundColor Cyan

$WorkspaceWin = Split-Path -Parent $MyInvocation.MyCommand.Path
Write-Host "Windows Workspace: $WorkspaceWin"

# Convert Windows path to WSL path
$Drive = $WorkspaceWin.Substring(0, 1).ToLower()
$PathWithoutDrive = $WorkspaceWin.Substring(2).Replace('\', '/')
$WorkspaceWsl = "/mnt/$Drive$PathWithoutDrive"

Write-Host "Launching multi-seed NS-3 batch simulation runs inside WSL2..." -ForegroundColor Yellow
wsl -e bash -c "chmod +x '$WorkspaceWsl/run_experiments.sh' && '$WorkspaceWsl/run_experiments.sh'"

Write-Host "`nRunning Matplotlib Analysis Figures in Windows environment..." -ForegroundColor Yellow
if (Get-Command python -ErrorAction SilentlyContinue) {
    python "$WorkspaceWin\plot_results.py"
}

Write-Host "`nAll simulation runs and figure outputs completed successfully!" -ForegroundColor Green
