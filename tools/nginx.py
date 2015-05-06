#!/usr/bin/env python3

import json
import os
import shutil
import socket
import string


def gethostname():
    hostname = socket.gethostname()
    i = hostname.find(".")
    if i == -1:
        return hostname
    else:
        return hostname[:i]


def setup_nginx_config():
    host_config_root = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "brewery", "web", "nginx")
    host_config_filename = "config-{}.json".format(gethostname())
    host_config_file = os.path.join(host_config_root, host_config_filename)
    host_config_file_default = os.path.join(host_config_root, "config-default.json")
    host_config_file_template = os.path.join(host_config_root, "nginx.conf.template")
    host_config_file_target = "/tmp/nginx/nginx.conf"

    print("Host config file: {}".format(host_config_file))

    if not os.path.exists(host_config_file):
        print("Host config file not found using default: {}".format(host_config_file_default))
        host_config_file = host_config_file_default

    listen_port = -1
    try:
        tmp = string.Template(open(host_config_file_template).read())
        data = json.load(open(host_config_file))
        listen_port = data["listen_port"]
        rendered = tmp.substitute(data)
        f = open(host_config_file_target,"w")
        f.write(rendered)
        f.close()
    except Exception as e:
        print(e)

    return listen_port


if __name__ == "__main__":
    if os.path.exists("/tmp/nginx"):
        shutil.rmtree("/tmp/nginx")
    os.makedirs("/tmp/nginx")
    port = setup_nginx_config()
    print("Web interface available at http://localhost:{}".format(port))
    print("Press CTRL-C to interrupt")
    os.execlp("nginx", "nginx", "-c", "/tmp/nginx/nginx.conf", "-g", "daemon off;")
