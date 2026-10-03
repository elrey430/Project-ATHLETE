#!/bin/bash
# Project ATHLETE - starts the job's run.sh detached from the SSH session (GpuJob.ps1 calls this).
# When run.sh ends: results.tgz holds results/ and job.log, and DONE holds run.sh's exit code.
cd "$(dirname "$0")"
if [ -f STARTED ]; then echo "already started"; exit 0; fi  # GpuJob.ps1 may retry this call
touch STARTED
sed -i 's/\r$//' ./*.sh ./*.py 2>/dev/null || true  # files copied from Windows may have CRLF endings
mkdir -p results
nohup setsid bash -c 'bash run.sh > job.log 2>&1; code=$?; tar czf results.tgz results job.log; echo $code > DONE' > /dev/null 2>&1 < /dev/null &
echo "started"
