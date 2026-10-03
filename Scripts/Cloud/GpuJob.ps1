# Project ATHLETE - runs one job on a cloud VM (default: one L4 GPU) and brings the results home.
# Usage:  powershell -ExecutionPolicy Bypass -File Scripts\Cloud\GpuJob.ps1 -Job <name> [-MaxRunDuration 80m]
#             [-ExtraFiles a,b] [-SpotAttempts 6] [-LiveFiles results/agent/x.pkl,...] [-OnDemand]
#
# A job is a folder in Scripts\Cloud\jobs\ with a run.sh (and whatever it needs). The VM runs it DETACHED
# from SSH, so a dropped connection can't kill a long training run. Every -PollSeconds this script prints
# the job's new log lines and checks that Google hasn't reclaimed the VM. When the job ends it copies
# results.tgz (the job's results\ folder plus its log) to Saved\Cloud\<run>\ and deletes the VM.
#
# Spot first, then on-demand: Spot VMs cost ~40% less but may be sold out or reclaimed. One ATTEMPT = a
# round of all zones without a free Spot VM, or a Spot VM reclaimed mid-job. After -SpotAttempts
# attempts the run continues on-demand (which Google doesn't reclaim). -OnDemand skips Spot entirely.
#
# Live files (-LiveFiles, paths under the job folder, e.g. a training checkpoint) are copied home to
# Saved\Cloud\<run>\live\ whenever they change. If a VM is lost, the next VM gets them back in place
# before the job starts, so a job that resumes from them (e.g. athlete-walk) loses at most one chunk.
#
# -CreateImageFamily <family>: when the job succeeds, the VM's disk becomes a new image in that family
# (BuildImage.ps1 uses this). Cost protection doesn't depend on this script: every VM has a maximum run
# time after which Google deletes it, disk included. VMs have no public IP (SSH through IAP; outbound
# internet through Cloud NAT, see SetupCloudNetwork.ps1) and no service account.
#
# -AttachRun <run> -AttachVm <vm> -AttachZone <zone>: follow a job that is already running (e.g. after
# this script was killed: 2026-10-01): copy live files, fetch the results, delete the VM. Nothing is
# created, uploaded or started. Start long runs detached (Start-Process), not as a child of a shell with
# a time limit.

param(
    [Parameter(Mandatory)] [string]$Job,
    [string]$MaxRunDuration = '1h',
    [string[]]$ExtraFiles = @(),
    [string[]]$LiveFiles = @(),
    [int]$PollSeconds = 60,
    [int]$SpotAttempts = 6,
    [int]$RetryMinutes = 10,
    [int]$OnDemandRounds = 6,
    [string]$MachineType = $null,
    [int]$DiskGb = 100,
    [string[]]$Image = $null, # @(family, project); default: our pre-installed image if it exists

    [string]$CreateImageFamily = $null,
    # A previous run's live folder (Saved\Cloud\<run>\live): its files go to the first VM, to resume from.
    [string]$ResumeFrom = $null,
    [switch]$OnDemand,
    [switch]$KeepVm,
    [string]$AttachRun = $null,
    [string]$AttachVm = $null,
    [string]$AttachZone = $null
)

. (Join-Path $PSScriptRoot 'CloudEnv.ps1')
if (-not $MachineType) { $MachineType = $GpuMachineType }

$JobDir = Join-Path $PSScriptRoot "jobs\$Job"
if (-not (Test-Path (Join-Path $JobDir 'run.sh'))) { throw "No job '$Job' ($JobDir\run.sh not found)" }
# The job's files, the shared ones (jobs\_*), and -ExtraFiles (files or folders, relative to the project).
$Files = @(Get-ChildItem $JobDir -File | ForEach-Object FullName) + @(Get-ChildItem (Join-Path $PSScriptRoot 'jobs') -File -Filter '_*' | ForEach-Object FullName)
foreach ($Extra in $ExtraFiles) {
    $Path = if ([IO.Path]::IsPathRooted($Extra)) { $Extra } else { Join-Path $ProjectRoot $Extra }
    if (-not (Test-Path $Path)) { throw "Extra file not found: $Path" }
    $Files += $Path
}
# Upload list as (local file, remote folder). Folders are copied file by file: gcloud's --recurse passes
# an option PuTTY's pscp (gcloud's copier on Windows) rejects (2026-09-30).
$Uploads = @()
foreach ($File in $Files) {
    if (Test-Path $File -PathType Container) {
        $Root = Split-Path (Resolve-Path $File) -Parent
        foreach ($Item in Get-ChildItem $File -Recurse -File | Where-Object { $_.FullName -notmatch '\\__pycache__\\' }) {
            $Uploads += , @($Item.FullName, ('job/' + (Split-Path $Item.FullName.Substring($Root.Length + 1) -Parent).Replace('\', '/')))
        }
    } else {
        $Uploads += , @($File, 'job')
    }
}

$RunName = if ($AttachRun) { $AttachRun } else { 'athlete-' + ($Job -replace '[^a-z0-9-]', '-') + '-' + (Get-Date -Format 'MMdd-HHmm') }
if ($AttachRun -and -not ($AttachVm -and $AttachZone)) { throw "-AttachRun needs -AttachVm and -AttachZone" }
$ResultsDir = Join-Path $ProjectRoot "Saved\Cloud\$RunName"
$LiveDir = Join-Path $ResultsDir 'live'
$Started = Get-Date
$UsedNames = @()
$Name = $null
$Zone = $null
$UseSpot = (-not $OnDemand) -and $SpotAttempts -gt 0
$AttemptsUsed = 0
$OnDemandTries = 0

function Copy-Up([string]$Local, [string]$RemoteFolder) {
    # IAP tunnels occasionally drop a transfer ("Remote side unexpectedly closed network connection",
    # 2026-09-30): retry each file a few times.
    for ($Try = 1; $true; ++$Try) {
        & $Gcloud compute scp $Local "${Name}:$RemoteFolder/" @(Get-SshArgs $Zone)
        if ($LASTEXITCODE -eq 0) { return }
        if ($Try -ge 3) { throw "Upload of $Local failed 3 times" }
        Write-Warning "Upload of $(Split-Path $Local -Leaf) failed (attempt $Try); retrying"
        Start-Sleep -Seconds 10
    }
}

function Use-Attempt([string]$Why) {
    $script:AttemptsUsed++
    Write-Warning "Spot attempt $script:AttemptsUsed of ${SpotAttempts}: $Why"
    if ($script:AttemptsUsed -ge $SpotAttempts) {
        $script:UseSpot = $false
        Write-Host "Spot attempts used up: continuing on-demand" -ForegroundColor Yellow
    }
}

try {
    New-Item -ItemType Directory -Force -Path $LiveDir | Out-Null
    if ($ResumeFrom) {
        if (-not (Test-Path $ResumeFrom)) { throw "Resume folder not found: $ResumeFrom" }
        Copy-Item (Join-Path $ResumeFrom '*') $LiveDir -Recurse -Force
        Write-Host "Resuming from $ResumeFrom" -ForegroundColor Cyan
    }
    $ExitCode = $null
    while ($null -eq $ExitCode) {
        # ---- Get a VM ----
        $Zone = $null
        $Attached = $AttachVm -and $UsedNames.Count -eq 0
        if ($Attached) { $Name = $AttachVm; $Zone = $AttachZone; $UsedNames += $Name }
        while (-not $Zone) {
            $Name = "$RunName-v$($UsedNames.Count + 1)"
            $UsedNames += $Name
            $Zone = New-GpuVm $Name $MaxRunDuration -OnDemand:(-not $UseSpot) -MachineType $MachineType -DiskGb $DiskGb -Image $Image
            if ($Zone) { break }
            if ($UseSpot) {
                Use-Attempt "no Spot $MachineType in any zone"
                if ($UseSpot) { Write-Host "Trying again in $RetryMinutes min" -ForegroundColor Yellow; Start-Sleep -Seconds (60 * $RetryMinutes) }
            } else {
                if (++$OnDemandTries -ge $OnDemandRounds) { throw "No on-demand $MachineType in any zone after $OnDemandRounds rounds" }
                Write-Host "No on-demand $MachineType either; trying again in $RetryMinutes min" -ForegroundColor Yellow
                Start-Sleep -Seconds (60 * $RetryMinutes)
            }
        }
        if ($Zone -isnot [string] -or $Zone -notin $GcpZones) { throw "Internal error: New-GpuVm returned '$Zone', not a zone" }
        $Ssh = Get-SshArgs $Zone

        if ($Attached) {
            Write-Host "Attached to $Name ($Zone): following the job already running there" -ForegroundColor Cyan
        } else {
            # ---- Upload the job, and any live files from a previous VM (to resume from) ----
            $Remote = @($Uploads)
            foreach ($Live in $LiveFiles) {
                $Local = Join-Path $LiveDir $Live
                if (Test-Path $Local) { $Remote += , @($Local, ('job/' + (Split-Path $Live -Parent).Replace('\', '/')).TrimEnd('/')) }
            }
            $Folders = @($Remote | ForEach-Object { $_[1] } | Sort-Object -Unique)
            Invoke-Remote $Name $Zone ('mkdir -p ' + ($Folders -join ' '))
            foreach ($Upload in $Remote) { Copy-Up $Upload[0] $Upload[1] }
            Invoke-Remote $Name $Zone 'bash job/_launch.sh' # safe to retry: _launch.sh starts the job only once
            Write-Host "Job '$Job' running on $Name ($Zone); new log lines every $PollSeconds s" -ForegroundColor Cyan
        }

        # ---- Follow it: log lines, live files, DONE, or a lost VM ----
        $Offset = 1
        $Seen = @{}
        $Lost = $false
        while ($null -eq $ExitCode) {
            Start-Sleep -Seconds $PollSeconds
            if (-not (Get-VmStatus $Name $Zone)) { $Lost = $true; break }
            $Poll = & $Gcloud compute ssh $Name @Ssh --command "bash job/_poll.sh $Offset $($LiveFiles -join ' ')" 2>$null
            if ($LASTEXITCODE -ne 0) { Write-Warning "Poll failed (network?); retrying"; continue }
            $Changed = @()
            foreach ($Line in $Poll) {
                if ($Line -match '^##LINES (\d+)$') { $Offset = [int]$Matches[1] + 1 }
                elseif ($Line -match '^##DONE (\d+)$') { $ExitCode = [int]$Matches[1] }
                elseif ($Line -match '^##FILE (\S+) (\d+)$') { if ($Seen[$Matches[1]] -ne $Matches[2]) { $Changed += , @($Matches[1], $Matches[2]) } }
                else { Write-Host $Line }
            }
            foreach ($File in $Changed) {
                $Target = Join-Path $LiveDir (Split-Path $File[0] -Parent)
                New-Item -ItemType Directory -Force -Path $Target | Out-Null
                & $Gcloud compute scp "${Name}:job/$($File[0])" $Target @Ssh 2>$null | Out-Null
                if ($LASTEXITCODE -eq 0) { $Seen[$File[0]] = $File[1]; Write-Host "  (copied home: $($File[0]))" -ForegroundColor DarkGray }
            }
        }
        if ($Lost) {
            if (-not $UseSpot) { throw "$Name (on-demand) disappeared before the job finished" }
            Use-Attempt "$Name was reclaimed by Google mid-job"
            continue # next VM; it resumes from the live files copied so far
        }
    }

    # ---- Results ----
    Invoke-Gcloud compute scp "${Name}:job/results.tgz" $ResultsDir @Ssh
    # Windows' own tar: a GNU tar earlier on PATH (Git's) reads 'C:' as a remote host (2026-10-01).
    & (Join-Path $env:SystemRoot 'System32\tar.exe') -xzf (Join-Path $ResultsDir 'results.tgz') -C $ResultsDir
    $Minutes = ((Get-Date) - $Started).TotalMinutes
    if ($ExitCode -ne 0) { throw "Job '$Job' failed on the VM (exit $ExitCode) after $([int]$Minutes) min; log and results in $ResultsDir" }
    Write-Host ("Job '{0}' finished in {1:N0} min on {2} VM(s), {3} Spot attempt(s) used; results in {4}" -f $Job, $Minutes, $UsedNames.Count, $AttemptsUsed, $ResultsDir) -ForegroundColor Green

    if ($CreateImageFamily) {
        Write-Host "Creating an image in family '$CreateImageFamily' from $Name's disk" -ForegroundColor Cyan
        # The job folder holds this job's STARTED/DONE markers: left in the image, every job on it would
        # think it had already run.
        Invoke-Remote $Name $Zone 'rm -rf job'
        Invoke-Gcloud compute instances stop $Name --zone $Zone --project $GcpProject --quiet
        $ImageName = "$CreateImageFamily-" + (Get-Date -Format 'yyyyMMdd-HHmm')
        Invoke-Gcloud compute images create $ImageName --project $GcpProject --source-disk $Name --source-disk-zone $Zone --family $CreateImageFamily
        Write-Host "Image $ImageName ready (family $CreateImageFamily)" -ForegroundColor Green
    }
}
finally {
    foreach ($Used in $UsedNames) {
        if ($KeepVm -and $Used -eq $Name -and $Zone -and (Get-VmStatus $Name $Zone)) {
            Write-Warning "Keeping $Name; Google deletes it after $MaxRunDuration. Delete it sooner with: gcloud compute instances delete $Name --zone $Zone --project $GcpProject"
        } else {
            Remove-GpuVm $Used $MaxRunDuration
        }
    }
}
