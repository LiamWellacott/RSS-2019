import json
import numpy as np
import matplotlib.pyplot as plt

def _timesAt(times, i):
    ''' the list of times at the ith step '''
    return([ t[i] for t in times])

def _averageTime(times, i):
    ''' returns average time taken up to the ith point'''
    return sum(_timesAt(times, i)) / len(times)

def averagePlanningTime(times):
    '''average time from robot start to begin moving'''
    print("Average planning time: %s" % (_averageTime(times,0),))

def averageOverallTime(times):
    '''average time to smashing the button'''
    print("Average overall time: %s" % (_averageTime(times,-1),))

def _getdists(actual, observed):
    ''' calculate distances from actual, yaw ignored because it wasn't measured '''
    dists = []
    for i in range(len(actual)):
        diff = np.array(observed[i][:-1]) - np.array(actual[i][:-1])
        dists.append(np.linalg.norm(diff))
    return dists

def averageDistanceFromActual(actual, observed):
    ''' measure the distance from actual point in each trial, report the average 
    '''
    dists = _getdists(actual, observed)
    print("Average Distance: %s" % (sum(dists) / len(dists),))

def distanceBoxplot(actual, observed):
    dists = _getdists(actual, observed)
    plt.boxplot(dists, labels=["Distance (m)"])
    plt.savefig("dist_button.png")
    plt.show()

def timeToCompleteBoxplot(times):
    ts_at_end = _timesAt(times, -1)
    plt.boxplot(ts_at_end, labels=["Time (s)"])
    plt.savefig("time_button.png")
    plt.show()

if __name__ == "__main__":

    # Load in button data
    with open("buttontest.json") as in_file:
        jcontent = json.load(in_file)

    # load in json data
    position_actual = [ d["position"]["actual"] for d in jcontent["data"]]
    position_reported = [ d["position"]["reported"] for d in jcontent["data"]]
    time_data = [ d["times"] for d in jcontent["data"]]
    
    # print stuff
    averagePlanningTime(time_data)
    averageOverallTime(time_data)
    averageDistanceFromActual(position_actual, position_reported)

    distanceBoxplot(position_actual, position_reported)
    timeToCompleteBoxplot(time_data)