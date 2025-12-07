# Convert Mermaid Diagrams to PNG
# Requires: @mermaid-js/mermaid-cli (mmdc)
#
# Installation:
#   npm install -g @mermaid-js/mermaid-cli
#
# Usage:
#   .\convert_mermaid_to_png.ps1
#
# Output:
#   PNG files created in same directory as .mmd files

Write-Host "🎨 Mermaid to PNG Converter" -ForegroundColor Cyan
Write-Host "==========================" -ForegroundColor Cyan
Write-Host ""

# Check if mmdc is installed
$mmdcPath = Get-Command mmdc -ErrorAction SilentlyContinue

if (-not $mmdcPath) {
    Write-Host "❌ Error: mmdc (Mermaid CLI) not found" -ForegroundColor Red
    Write-Host ""
    Write-Host "Install with:"
    Write-Host "  npm install -g @mermaid-js/mermaid-cli"
    Write-Host ""
    Write-Host "Or use Docker:"
    Write-Host '  docker run --rm -v ${PWD}:/data minlag/mermaid-cli -i /data/input.mmd -o /data/output.png'
    exit 1
}

# Get mmdc version
$version = & mmdc --version 2>&1
Write-Host "✓ mmdc found: $version" -ForegroundColor Green
Write-Host ""

# Base directory
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$baseDir = Split-Path -Parent (Split-Path -Parent $scriptDir)
$diagramDir = Join-Path $baseDir "assets\diagrams\architecture"

Write-Host "Converting diagrams..." -ForegroundColor Yellow
Write-Host ""

# Array of diagrams to convert
$diagrams = @(
    "nominal-flow",
    "navigation-failure",
    "object-not-found"
)

$successCount = 0
$failCount = 0

foreach ($diagram in $diagrams) {
    $input = Join-Path $diagramDir "$diagram.mmd"
    $output = Join-Path $diagramDir "$diagram.png"

    if (-not (Test-Path $input)) {
        Write-Host "  ⚠  Warning: $input not found, skipping" -ForegroundColor Yellow
        $failCount++
        continue
    }

    Write-Host "  Converting $diagram.mmd → $diagram.png ... " -NoNewline

    try {
        # Convert with mmdc
        $result = & mmdc -i $input -o $output -b transparent -t dark -w 1200 2>&1

        if (Test-Path $output) {
            Write-Host "✓" -ForegroundColor Green
            $successCount++
        } else {
            Write-Host "✗" -ForegroundColor Red
            $failCount++
        }
    } catch {
        Write-Host "✗" -ForegroundColor Red
        Write-Host "    Error: $_" -ForegroundColor Red
        $failCount++
    }
}

Write-Host ""
Write-Host "=========================="
Write-Host "Summary:"
Write-Host "  ✓ Success: $successCount" -ForegroundColor Green

if ($failCount -gt 0) {
    Write-Host "  ✗ Failed: $failCount" -ForegroundColor Red
}

Write-Host ""

if ($successCount -gt 0) {
    Write-Host "Output directory: $diagramDir"
    Write-Host ""
    Write-Host "Generated PNGs:"
    Get-ChildItem -Path $diagramDir -Filter "*.png" | Format-Table Name, Length, LastWriteTime -AutoSize
}
