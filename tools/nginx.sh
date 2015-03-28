#/bin/sh
DIR=`dirname $0`
cd $DIR
CONFIG_FILE=$PWD/../brewery/web/nginx.conf
LISTEN_PORT=$(grep listen ${CONFIG_FILE} | grep -o "[0-9]\+")
echo "Listening on port ${LISTEN_PORT}"
echo "Press CTRL-C to interrupt"
exec nginx -c ${CONFIG_FILE} -g "daemon off;"
