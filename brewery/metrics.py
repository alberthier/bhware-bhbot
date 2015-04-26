import greplin
import greplin.scales
from greplin import scales
from greplin.scales.formats import jsonFormat

STATS = scales.collection('/goaldecider',
    scales.IntStat('errors'),
    scales.IntStat('success'),
    scales.PmfStat('duration')
    )

# In a request handler

STATS.success += 1


def write(filename):
    scales.dumpStatsTo(filename)