#!/bin/sh
if [ -z "brew ls --versions git-ftp" ]; then
	brew install git-ftp
fi
git ftp push -u lightgraffiti.de ftp://ftp.lightgraffiti.de/lg -k
