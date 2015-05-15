import random
import packets
import logger

from definitions import *

def safe_say(state, category):
    try:
        state.event_loop.send_packet(packets.Say(random.choice(TEXTS[category])))
    except Exception as e:
        logger.error("safe_say: {}".format(e))
