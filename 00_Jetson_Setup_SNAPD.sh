#!/usr/bin/env bash
# ---- After a fresh Installation of Jetpack Snapd -> Issue Snapd Applications wont Start
# Simple fast fix is to use an older Snapd Version 

sudo snap download snapd --revision=24724
sudo snap ack snapd_24724.assert
sudo snap install snapd_24724.snap
sudo snap refresh --hold snapd

# Apps like Chromium or Firefox should now Start
