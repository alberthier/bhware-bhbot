import json
import logger

import os
import shutil
import socket
import string

def gethostname():
    hostname_file = os.path.join(os.path.dirname(__file__),"../.hostname")
    if os.path.exists(hostname_file):
        return open(hostname_file).read().strip()
    return socket.gethostname()

def setup_nginx_config():
    host_config_root = os.path.join(os.path.dirname(__file__), "web")
    host_config_filename = "nginx-config-{}.json".format(gethostname())
    host_config_file = os.path.join(host_config_root, host_config_filename)
    host_config_file_default = "web/nginx-config-default.json"
    host_config_file_template = os.path.join(host_config_root, "nginx.conf.template")
    host_config_file_target = os.path.join(host_config_root, "nginx.conf")

    logger.log("Host config file: {}".format(host_config_file))

    if not os.path.exists(host_config_file):
        logger.log("Host config file not found using default: {}".format(host_config_file_default))
        host_config_file=host_config_file_default

    try:
        tmp=string.Template(open(host_config_file_template).read())
        data=json.load(open(host_config_file))
        rendered=tmp.substitute(data)
        open(host_config_file_target,"w").write(rendered)

    except Exception as e:
        logger.log_exception(e)