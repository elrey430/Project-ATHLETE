#!/bin/bash
# Project ATHLETE - GpuJob.ps1 calls this every poll. Prints the job log from line $1 on, then
# "##LINES <total>"; "##FILE <path> <mtime>" for each live file given after $1 that exists; and
# "##DONE <code>" once the job has ended.
cd "$(dirname "$0")"
if [ -f job.log ]; then
    tail -n +"$1" job.log
    echo "##LINES $(wc -l < job.log)"
else
    echo "##LINES 0"
fi
for file in "${@:2}"; do
    if [ -f "$file" ]; then echo "##FILE $file $(stat -c %Y "$file")"; fi
done
if [ -f DONE ]; then echo "##DONE $(cat DONE)"; fi
