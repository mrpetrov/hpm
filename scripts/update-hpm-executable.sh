#!/bin/bash

daemon=hpm
daemon_backup=hpm_backup
daemon_pid=/run/$daemon.pid
log_file=/run/shm/$daemon.log

echo "Trying to kill running daemon $daemon..."
if [ ! -e $daemon_pid ]
then
    killall -e $daemon
else
    kill `cat $daemon_pid`
    rm -f $daemon_pid
fi
sleep 1
/etc/cron.hourly/move-hpm-log
echo "Replacing /usr/sbin/$daemon with the one from /home/pi/$daemon..."
cp /home/pi/$daemon/$daemon /usr/sbin
echo "Replacing $daemon-reload and $daemon-restart in /usr/sbin with ones from /home/pi/$daemon/scripts/..."
cp /home/pi/$daemon/scripts/$daemon-reload /usr/sbin
cp /home/pi/$daemon/scripts/$daemon-restart /usr/sbin
cp /home/pi/$daemon/scripts/$daemon-stop /usr/sbin
cp /home/pi/$daemon/scripts/$daemon_backup-cfg /usr/sbin
chmod +x /usr/sbin/$daemon-reload
chmod +x /usr/sbin/$daemon-restart
chmod +x /usr/sbin/$daemon-stop
chmod +x /usr/sbin/$daemon_backup-cfg
sleep 1
echo "Starting $daemon again..."
/usr/sbin/$daemon
echo "Here is the log:"
tail -n 30 $log_file
