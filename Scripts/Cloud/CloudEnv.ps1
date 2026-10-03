# Project ATHLETE - shared settings for the cloud GPU scripts. Dot-source this file; don't run it directly.
# Override with the environment variables ATHLETE_GCP_PROJECT and ATHLETE_GCP_ZONES (comma-separated).

. (Join-Path $PSScriptRoot '..\AthleteEnv.ps1')
# Native tools (gcloud) report progress on stderr; failures are checked through exit codes instead.
$ErrorActionPreference = 'Continue'

$GcpProject = if ($env:ATHLETE_GCP_PROJECT) { $env:ATHLETE_GCP_PROJECT } else { 'project-eb44c2f3-0e73-4188-890' } # "Project ATHLETE"
# Zones tried in order. All in us-central1, where SetupCloudNetwork.ps1 put the Cloud NAT.
$GcpZones = if ($env:ATHLETE_GCP_ZONES) { $env:ATHLETE_GCP_ZONES.Split(',') } else { @('us-central1-c', 'us-central1-a', 'us-central1-b') }

# One NVIDIA L4 (24 GB), 8 vCPUs, 32 GB RAM. Ubuntu with the NVIDIA driver and CUDA preinstalled.
# Ubuntu 24.04 brings Python 3.12 (current JAX needs 3.11+; 22.04 has 3.10).
$GpuMachineType = 'g2-standard-8'
$GpuImageFamily = 'common-cu129-ubuntu-2404-nvidia-580'
$GpuImageProject = 'deeplearning-platform-release'
# That image plus everything the jobs install (LocoMuJoCo stack, render libraries, motion data), built by
# BuildImage.ps1 in this project. Used automatically when it exists.
$AthleteImageFamily = 'athlete-gpu'

# Always gcloud.cmd, never the SDK's gcloud.ps1 (which `Get-Command gcloud` picks when scripts may run):
# the .ps1 passes an inline array argument such as @(Get-SshArgs $Zone) to gcloud as ONE string, so every
# SSH attempt failed with "unrecognized arguments" (2026-10-01).
$Gcloud = Get-Command gcloud -All -ErrorAction SilentlyContinue | Where-Object { $_.Source -like '*.cmd' } |
    Select-Object -First 1 -ExpandProperty Source
if (-not $Gcloud) { $Gcloud = Join-Path $env:LOCALAPPDATA 'Google\Cloud SDK\google-cloud-sdk\bin\gcloud.cmd' }
if (-not (Test-Path $Gcloud)) { throw "gcloud not found. Install the Google Cloud CLI." }

function Invoke-Gcloud {
    & $Gcloud @args
    if ($LASTEXITCODE -ne 0) { throw "gcloud $($args[0..2] -join ' ') failed ($LASTEXITCODE)" }
}

function Get-SshArgs([string]$Zone) {
    # No public IP (organization policy): SSH goes through Identity-Aware Proxy.
    return @('--zone', $Zone, '--project', $GcpProject, '--tunnel-through-iap', '--quiet', '--strict-host-key-checking=no')
}

function Get-VmStatus([string]$Name, [string]$Zone) {
    # RUNNING, STAGING, ...; empty if the VM doesn't exist (never created, or reclaimed and deleted).
    $Status = & $Gcloud compute instances describe $Name --zone $Zone --project $GcpProject --format='value(status)' 2>$null
    if ($LASTEXITCODE -ne 0) { return '' }
    return "$Status".Trim()
}

function Wait-ForSsh([string]$Name, [string]$Zone) {
    # $true when SSH works, $false if Google reclaimed the VM; throws if SSH never comes up.
    # Needs two answers in a row 10 s apart: on first boot the VM restarts SSH shortly after it first
    # answers, which dropped the next command (2026-09-30).
    $LastError = ''
    $Answers = 0
    for ($Try = 1; $Try -le 30; ++$Try) {
        $Status = Get-VmStatus $Name $Zone
        if ($Status -notin 'RUNNING', 'STAGING', 'PROVISIONING') {
            Write-Warning "$Name in $Zone was reclaimed by Google (status: '$Status')"
            return $false
        }
        $Output = & $Gcloud compute ssh $Name @(Get-SshArgs $Zone) --command 'echo ready' 2>&1
        if ($LASTEXITCODE -eq 0) {
            if (++$Answers -ge 2) { return $true }
            Start-Sleep -Seconds 10
            continue
        }
        $Answers = 0
        # The cause, without gcloud's generic help footer (which alone was logged on 2026-10-01).
        $LastError = ($Output | ForEach-Object { "$_".Trim() } |
            Where-Object { $_ -and $_ -notmatch 'help text|gcloud help|RemoteException|^At |^\+|CategoryInfo|FullyQualifiedErrorId' } |
            Select-Object -Last 4) -join ' | '
        Start-Sleep -Seconds 15
    }
    throw "SSH to $Name never came up. Last error: $LastError"
}

function Invoke-Remote([string]$Name, [string]$Zone, [string]$Command, [int]$Attempts = 3) {
    # A short remote command, retried: IAP tunnels occasionally drop a connection.
    for ($Try = 1; $true; ++$Try) {
        & $Gcloud compute ssh $Name @(Get-SshArgs $Zone) --command $Command
        if ($LASTEXITCODE -eq 0) { return }
        if ($Try -ge $Attempts) { throw "Remote command failed $Attempts times: $Command" }
        Write-Warning "Remote command failed (attempt $Try); retrying: $Command"
        Start-Sleep -Seconds 10
    }
}

function Get-DefaultImage {
    # Our pre-installed image family if it exists (Scripts\Cloud\BuildImage.ps1), else the plain Deep
    # Learning image (jobs then install their packages themselves, ~10 min and ~3.5 GiB through the NAT).
    & $Gcloud compute images describe-from-family $AthleteImageFamily --project $GcpProject --format='value(name)' 2>$null | Out-Null
    if ($LASTEXITCODE -eq 0) { return @($AthleteImageFamily, $GcpProject) }
    return @($GpuImageFamily, $GpuImageProject)
}

function New-GpuVm([string]$Name, [string]$MaxRunDuration, [switch]$OnDemand, [string]$MachineType = $GpuMachineType,
                   [string[]]$Image = $null, [int]$DiskGb = 100) {
    # Creates a VM (default: one L4 GPU) in the first zone that has one and keeps it running; returns that
    # zone, or $null. Cost protection: Google deletes the VM (disk included) after MaxRunDuration whatever
    # happens here. Spot VMs (default) cost ~40% less but can be reclaimed; both kinds get the time limit.
    $Provisioning = if ($OnDemand) { @('--provisioning-model', 'STANDARD') } else { @('--provisioning-model', 'SPOT') }
    if (-not $Image) { $Image = Get-DefaultImage }
    foreach ($Zone in $GcpZones) {
        Write-Host "Creating $Name ($MachineType, $(if ($OnDemand) { 'on-demand' } else { 'Spot' }), image $($Image[0]), deleted after $MaxRunDuration at the latest) in $Zone" -ForegroundColor Cyan
        & $Gcloud compute instances create $Name --project $GcpProject --zone $Zone `
            --machine-type $MachineType @Provisioning --instance-termination-action DELETE `
            --max-run-duration $MaxRunDuration --maintenance-policy TERMINATE `
            --image-family $Image[0] --image-project $Image[1] `
            --boot-disk-size "${DiskGb}GB" --boot-disk-type pd-balanced `
            --no-address --no-service-account --no-scopes | Out-Host # shown, not returned: this function returns only the zone
        if ($LASTEXITCODE -ne 0) { Write-Warning "No $MachineType available in $Zone"; continue }
        Write-Host "Waiting for SSH" -ForegroundColor Cyan
        if (Wait-ForSsh $Name $Zone) { return $Zone }
    }
    return $null
}

function Remove-GpuVm([string]$Name, [string]$MaxRunDuration) {
    # Deletes the VM wherever it still exists. Google deletes it after MaxRunDuration anyway.
    foreach ($Zone in $GcpZones) {
        if (-not (Get-VmStatus $Name $Zone)) { continue }
        Write-Host "Deleting $Name in $Zone" -ForegroundColor Cyan
        & $Gcloud compute instances delete $Name --zone $Zone --project $GcpProject --quiet
        if ($LASTEXITCODE -ne 0) { Write-Warning "Delete failed: check the console. Google still deletes it after $MaxRunDuration." }
    }
}
