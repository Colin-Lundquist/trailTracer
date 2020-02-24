import cProfile, pstats, io
from pstats import SortKey
pr = cProfile.Profile()
pr.enable()

#import Tracker.py
pr.run('python3 Tracker.py')

pr.disable()
s = io.StringIO()
sortby = SortKey.CUMULATIVE
ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
ps.print_stats()
print(s.getvalue())
