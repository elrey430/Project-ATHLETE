# Project ATHLETE - one-time network setup for the cloud GPU VMs. Safe to re-run.
# Usage:  powershell -ExecutionPolicy Bypass -File Scripts\Cloud\SetupCloudNetwork.ps1
#
# The organization policy (compute.vmExternalIpAccess) forbids public IPs on VMs, a secure default
# that stays in place. Instead:
#  - Identity-Aware Proxy (IAP): SSH reaches the VM through Google, after checking your Google login.
#    Needs the IAP API and a firewall rule for IAP's address range (35.235.240.0/20) on port 22.
#  - Cloud NAT: the VM can open connections OUT (to download Python packages); nothing can connect in.
#    Costs (Cloud Billing catalog, us-central1, checked 2026-09-29): its external IP $0.005/h ALWAYS,
#    about $3.60/month even with no VM; $0.0014 per VM-hour; $0.045 per GiB of traffic. Each run's
#    package install is ~3 GiB (~$0.14), more than a short run's GPU time.

. (Join-Path $PSScriptRoot 'CloudEnv.ps1')
$Region = $GcpZones[0] -replace '-[a-z]$', ''

function Test-Gcloud { & $Gcloud @args *> $null; return $LASTEXITCODE -eq 0 }

Write-Host "Enabling the Identity-Aware Proxy API" -ForegroundColor Cyan
Invoke-Gcloud services enable iap.googleapis.com --project $GcpProject

if (-not (Test-Gcloud compute firewall-rules describe athlete-allow-iap-ssh --project $GcpProject)) {
    Write-Host "Firewall: SSH from IAP only" -ForegroundColor Cyan
    Invoke-Gcloud compute firewall-rules create athlete-allow-iap-ssh --project $GcpProject --network default `
        --direction INGRESS --action ALLOW --rules tcp:22 --source-ranges 35.235.240.0/20
}

if (-not (Test-Gcloud compute routers describe athlete-router --region $Region --project $GcpProject)) {
    Write-Host "Cloud Router for $Region" -ForegroundColor Cyan
    Invoke-Gcloud compute routers create athlete-router --project $GcpProject --network default --region $Region
}
if (-not (Test-Gcloud compute routers nats describe athlete-nat --router athlete-router --region $Region --project $GcpProject)) {
    Write-Host "Cloud NAT (outbound only)" -ForegroundColor Cyan
    Invoke-Gcloud compute routers nats create athlete-nat --project $GcpProject --router athlete-router --region $Region `
        --auto-allocate-nat-external-ips --nat-all-subnet-ip-ranges
}
Write-Host "Network ready for GPU VMs in $Region" -ForegroundColor Green
