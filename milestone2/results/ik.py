import json
import numpy as np
import matplotlib.pyplot as plt

truepos = []
data = []

def printRMSEDimension():
    ''' 
    Root Mean squared error for each dimension (x, y, z) to show if any particular dimension was a problem. 
    
    Root mean square error used to remove pos/negative cancelling effect (mean square) and to make the error in the order of the data (root) 
    
    '''
    mses = []
    for dim in range(len(truepos)):
        total = sum([ (observed[dim]-truepos[dim])**2 for observed in data])
        mses.append(np.sqrt(total / float(len(data))))
    print("MSE: %s" % (mses))

def printAverageDistance():
    ''' Average of the distance between observed and true point to show average accuracy'''
    total = sum( [ np.linalg.norm(d - truepos) for d in data] )
    print ("Distance: %s" % (total/float(len(data)),) )

def outputBoxPlots():
    ''' box plot of the collected data '''
    dists = [ np.linalg.norm(d - truepos) for d in data]
    plt.boxplot(dists, labels=["Distance (m)"])
    plt.savefig("ik.png")
    plt.show()

if __name__ == "__main__":

    # Load in IK data
    with open("ik.json") as in_file:
        jcontent = json.load(in_file)

    # convert to meters because we didn't do that...
    truepos = np.array(jcontent["truepos"]) / 100.0
    data = [ np.array(d) / 100.0 for d in jcontent["Data"]]

    # Output interesting things
    printRMSEDimension()

    printAverageDistance()

    outputBoxPlots()

