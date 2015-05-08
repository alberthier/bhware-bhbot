import contextlib
import datetime
from greplin import scales

import logger

STATS = scales.collection('/goaldecider',
    scales.PmfStat('duration')
    )


def write(filename=None):
    try:
        if filename is None:
            filename=logger.filepath.replace(".py",".json")
        logger.dbg("Writing metrics to {}".format(filename))
        scales.dumpStatsTo(filename)
    except Exception as e:
        logger.dbg("Exception in stats writing")
        logger.log_exception(e)


@contextlib.contextmanager
def simple_timer(name):
    start_time = datetime.datetime.now()
    yield
    delta = datetime.datetime.now() - start_time
    logger.log("{} duration: {} s".format(name, delta.total_seconds()))
    return False


class Timer:
    def __init__(self,name):
        self.start_time=None
        self.duration=0.0
        self.name=name

    def __enter__(self):
        self.start_time=datetime.datetime.now()

    def __exit__(self, exc_type, exc_val, exc_tb):
        # noinspection PyTypeChecker
        self.duration=(datetime.datetime.now()-self.start_time).total_seconds()
        if self.name:
            logger.log("{} duration: {} s".format(self.name, self.duration))
        return False