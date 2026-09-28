# Project ATHLETE - run automated tests headlessly and print a summary.
# Usage:  powershell -ExecutionPolicy Bypass -File Scripts\RunTests.ps1 [-Filter Athlete.Core]
# Exit code 0 = all tests passed. The full report lands in Saved\Automation\Reports\index.json.
# Close the Unreal Editor first.

param(
    # Tests whose full name starts with this prefix are run.
    [string]$Filter = 'Athlete',
    # Also print Info lines of passing tests (measurements, studies).
    [switch]$ShowInfo
)

. "$PSScriptRoot\AthleteEnv.ps1"

$ReportDir = Join-Path $ProjectRoot 'Saved\Automation\Reports'
if (Test-Path $ReportDir) { Remove-Item -Recurse -Force $ReportDir }

# -NullRHI: no rendering (tests don't need a GPU window). -unattended: never block on dialogs.
& $EditorCmd $ProjectFile "-ExecCmds=Automation RunTests $Filter" "-TestExit=Automation Test Queue Empty" `
    "-ReportExportPath=$ReportDir" -unattended -nopause -nosplash -NullRHI -stdout "-log=AthleteTests.log" | Out-Null

$ReportFile = Join-Path $ReportDir 'index.json'
if (-not (Test-Path $ReportFile)) {
    Write-Host "No test report was produced. See Saved\Logs\AthleteTests.log" -ForegroundColor Red
    # Note: quote any -arg=value containing a '.', or Windows PowerShell 5.1 splits it in two.
    exit 2
}

# The report is written as UTF-8 with a BOM on some engine versions; strip it before parsing.
$Report = (Get-Content $ReportFile -Raw -Encoding UTF8).TrimStart([char]0xFEFF) | ConvertFrom-Json

foreach ($Test in $Report.tests) {
    $Color = if ($Test.state -eq 'Success') { 'Green' } else { 'Red' }
    Write-Host ("[{0,-7}] {1}" -f $Test.state, $Test.fullTestPath) -ForegroundColor $Color
    foreach ($Entry in $Test.entries) {
        if ($ShowInfo -or $Entry.event.type -ne 'Info' -or $Test.state -ne 'Success') {
            Write-Host ("            {0}: {1}" -f $Entry.event.type, $Entry.event.message)
        }
    }
}

$Total = @($Report.tests).Count
Write-Host ""
Write-Host "Passed: $($Report.succeeded)  Passed with warnings: $($Report.succeededWithWarnings)  Failed: $($Report.failed)  Not run: $($Report.notRun)  (Total: $Total)"

if ($Total -eq 0) { Write-Host "No tests matched '$Filter'." -ForegroundColor Red; exit 3 }
if ($Report.failed -gt 0 -or $Report.notRun -gt 0) { exit 1 }
exit 0
