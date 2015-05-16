import random
import packets
import logger

from definitions import *

def safe_say(state, category):
    try:
        data = random.choice(TEXTS[category])
        if data.startswith("MEDIA"):
            path=data.split(":")[1]
            state.event_loop.send_packet(packets.PlayMedia(path))
        else:
            state.event_loop.send_packet(packets.Say(data))
    except Exception as e:
        logger.error("safe_say: {}".format(e))
