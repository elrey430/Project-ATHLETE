# Project ATHLETE - builds the pre-installed VM image (family "athlete-gpu") that GPU jobs boot from.
# Usage:  powershell -ExecutionPolicy Bypass -File Scripts\Cloud\BuildImage.ps1
#
# Runs jobs\image-setup on a cheap Spot CPU VM (e2-standard-8, ~$0.08/h; nothing there needs a GPU),
# then saves its disk as a new image in the family. GpuJob.ps1 uses the newest image of the family
# automatically. Rebuild after changing package versions. Storage: ~$0.05 per GiB-month of the image
# (the Deep Learning base is most of it); delete old images of the family with `gcloud compute images delete`.

param([string]$MaxRunDuration = '45m')

. (Join-Path $PSScriptRoot 'CloudEnv.ps1')
# Always from Google's Deep Learning image, never from an older athlete-gpu image.
& (Join-Path $PSScriptRoot 'GpuJob.ps1') -Job image-setup -MachineType e2-standard-8 -DiskGb 60 -MaxRunDuration $MaxRunDuration `
    -Image $GpuImageFamily, $GpuImageProject -CreateImageFamily $AthleteImageFamily
