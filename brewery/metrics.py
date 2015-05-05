from greplin import scales

import logger

STATS = scales.collection('/goaldecider',
    scales.IntStat('errors'),
    scales.IntStat('success'),
    scales.PmfStat('duration')
    )

# In a request handler

STATS.success += 1


def write(filename=None):
    try:
        if filename is None:
            filename=logger.filepath.replace(".py",".json")
        logger.dbg("Writing metrics to {}".format(filename))
        scales.dumpStatsTo(filename)
    except Exception as e:
        logger.dbg("Exception in stats writing")
        logger.log_exception(e)