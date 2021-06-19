#!/bin/bash

socat /dev/ttyACM_rov,raw,echo=0 tcp-listen:8000,fork,reuseaddr



