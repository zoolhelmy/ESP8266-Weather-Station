#!/bin/sh

# --------------------------------------------------------------------
#
# Initial author: Zool Johan
# Created       : 27 Nov 2022
#
# Function      : Commit data to GitHub
#
# Run           : Run as daily cron
#               : https://openwrt.org/docs/guide-user/base-system/cron
#
# --------------------------------------------------------------------

# Variables
PATH_GITHUB="/etc/collectd/git"                                         #

# Github update function
update_github() {

	echo
	echo "Update github..."

	cd $PATH_GITHUB

	eval $(ssh-agent -s) && ssh-add ~/.ssh/id_ed25519 && ssh-add -l

	git config --global user.name "zoolhelmy"
	git config --global user.email "zool@zoolhelmy"
	git remote set-url origin git@github.com:zoolhelmy/ESP8266-Weather-Station.git
	git init

	# git pull
	## git pull origin main

	cd $PATH_GITHUB/ESP8266-Weather-Station/data
	git add *.txt

	cd $PATH_GITHUB/ESP8266-Weather-Station/images/graph
	git add *.png

	git commit -m "ESP8266 data update $(date)"

	git push
	git status

	eval $(ssh-agent -k)
	
}

# Main
main() {

	update_github
	
}

main

# EOF