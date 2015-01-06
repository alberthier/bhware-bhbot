#/bin/sh
DIR=`dirname $0`
cd $DIR
exec nginx -c $PWD/../brewery/web/nginx.conf -g "daemon off;"
