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