#!/bin/bash
#str2str -in ntrip://CRTNSKYRYSE:SKYRYSESURV@132.239.152.175:2103/EBMD_RTCM3 -out ntrips://:@localhost:6001/voly
str2str -in ntrip://CRTNSKYRYSE:SKYRYSESURV@132.239.152.175:2103/EBMD_RTCM3 -out ntrips://127.0.0.1:6001/voly
