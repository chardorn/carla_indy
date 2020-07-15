#Make two clusters of points!
def stat(lst):
    #Calculate mean and std deviation from list of points
    n = float(len(lst))
    mean = sum(lst) /n
    stdev = np.sqrt((sum(x*x for x in lst) / n)) - (mean * mean)
    return mean, stdev

def parse(lst):
    n = 2 #number of clusters
    cluster = []
    for i in lst:
        if len(cluster <= 1):
            cluster.append(1)
            continue
        mean, stdev = stat(cluster)
        if abs(mean - i) > n * stdev:
            yield cluster
            cluster[:] = []
        cluster.append(i)
    yield cluster