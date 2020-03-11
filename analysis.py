# Ben Webb
# Spring 2018
# CS 251 Project 6

import numpy as np
from scipy import stats
import scipy.cluster.vq as vq
import random
import data


def data_range(headers, data):
    # Finds the range of data
    retV = []
    for header in headers:
        column = data.get_data(headersInclude=[header])
        retV.append( [column.min(0)[0,0], column.max(0)[0,0]] )
    return (retV)

def mean(headers, data):
    # Finds the mean of data
    retV = []
    for header in headers:
        column = data.get_data(headersInclude=[header])
        retV.append( column.mean(0)[0,0] )
    return retV

def stdev(headers, data):
    # Finds the stdev of data
    retV = []
    for header in headers:
        column = data.get_data(headersInclude=[header])
        retV.append( column.std(0)[0,0] )
    return retV

def normalize_columns_separately(headers, data):
    # Normalize each column, separately by column
    datarange = data_range(headers, data)
    d = []
    for i in range(len(headers)):
        if headers[i] == '':
            column = np.matrix(np.zeros(data.get_num_points())).T
        else:
            column = data.get_data(headersInclude=[headers[i]])
            for j in range(len(column)):
                column[j] = (column[j] - datarange[i][0])/(datarange[i][1]-datarange[i][0])
        d.append(column)
    return np.hstack(d)

def normalize_columns_together(headers, data):
    # Normalize all columns together
    mati = data.get_data(headersInclude=headers)
    min = np.matrix(data_range(headers, data)).min()
    max = np.matrix(data_range(headers, data)).max()
    for j in range(len(mati)):
        mati[j] = (mati[j] - min)/(max-min)
    return mati

def single_linear_regression(data_obj, ind_var, dep_var):
    # Run a line linear regression
    data_points = data_obj.get_data(headersInclude=[ind_var, dep_var])
    sl, yi, rv, pv, se = stats.linregress(data_points)
    return (sl, yi, rv, pv, se, (data_points.min(0)[0,0],data_points.min(0)[0,1]),(data_points.max(0)[0,0],data_points.max(0)[0,1]))

def linear_regression(d, ind, dep):
    # Run a multiple linear regression
    y = d.get_data([dep])
    A = np.hstack((d.get_data(ind),np.ones((d.get_num_points(),1))))
    AAinv = np.linalg.inv( np.dot(A.T, A) )
    x = np.linalg.lstsq( A, y )
    b = x[0]
    N = y.shape[0]
    C = b.shape[0]
    df_e = N-C
    df_r = C-1
    error = y - np.dot(A, b)
    sse = np.dot(error.T, error) / df_e
    stderr = np.sqrt( np.diagonal( sse[0, 0] * AAinv ) )
    t = b.T / stderr
    p = 2*(1 - stats.t.cdf(abs(t), df_e))
    r2 = 1 - error.var() / y.var()
    return (b, sse, r2, t, p)


# This version uses SVD
def pca(d, headers, normalize=True):

    if normalize:
        A = normalize_columns_separately(headers, d)
    else:
        A = d.get_data(headers)
    # assign to m the mean values of the columns of A
    m = np.mean(A, axis = 0)
    # assign to D the difference matrix A - m
    D = A - m
    # assign to U, S, V the result of running np.svd on d
    U, S, V = np.linalg.svd(D, full_matrices=False)
    eVals = np.square(S)/(D.shape[0]-1)
    pmat = (V * D.T).T
    return data.PCAData(pmat, V, eVals, m, headers)


def kmeans_numpy(d, headers, K, whiten=True):
    '''Takes in a Data object, a set of headers, and the number of clusters to create
    Computes and returns the codebook, codes, and representation error.
    '''
    A = d.get_data(headers)
    W = vq.whiten(A)
    codebook, bookerror = vq.kmeans(W, K)
    codes, error = vq.vq(W, codebook)


    return codebook, codes, error

# Selects K random rows from the data matrix A and returns them as a matrix
def kmeans_init(A, K):
    indices = list(range(A.shape[0]))
    random.shuffle(indices)
    return A[indices[:K],:]

# Given a data matrix A and a set of means in the codebook
# Returns a matrix of the id of the closest mean to each point
# Returns a matrix of the sum-squared distance between the closest mean and each point
def kmeans_classify(A, codebook):
    ssd = np.matrix(np.zeros((A.shape[0],codebook.shape[0])))
    for i in range(A.shape[0]):
        means = codebook - A[i, :]
        ssd[i,:] = np.sqrt(np.sum(np.square(means),axis=1)).T
    return np.argmin(ssd, axis=1), np.min(ssd, axis=1)

# Given a data matrix A and a set of K initial means, compute the optimal
# cluster means for the data and an ID and an error for each data point
def kmeans_algorithm(A, means):
    # set up some useful constants
    MIN_CHANGE = 1e-7     # might want to make this an optional argument
    MAX_ITERATIONS = 100  # might want to make this an optional argument
    D = means.shape[1]    # number of dimensions
    K = means.shape[0]    # number of clusters
    N = A.shape[0]        # number of data points

    for i in range(MAX_ITERATIONS):
        (codes, distances) = kmeans_classify(A, means)
        # codes[j,0] is the id of the closest mean to point j
        newmeans = np.zeros_like(means)
        counts = np.zeros(K).T
        for j in range(N):
            newmeans[codes[j,0]] += A[j]
            counts[codes[j,0]] += 1
        # finish calculating the means, taking into account possible zero counts
        for j in range(K):
            if counts[j] != 0:
                newmeans[j] = newmeans[j]/counts[j]
            else:
                newmeans[j] = kmeans_init(A, 1)
        # test if the change is small enough and exit if it is
        diff = np.sum(np.square(means - newmeans))
        means = newmeans
        if diff < MIN_CHANGE:
            break
    # call kmeans_classify one more time with the final means
    codes, errors = kmeans_classify( A, means )

    # return the means, codes, and errors
    return (means, codes.T, errors)


def kmeans(d, headers, K, whiten=True):
    '''Takes in a Data object, a set of headers, and the number of clusters to create
    Computes and returns the codebook, codes and representation errors.
    '''
    A = d.get_data(headers)
    if whiten:
        W = vq.whiten(A)
    else:
        W = A
    codebook = kmeans_init(W, K)
    codebook, codes, errors = kmeans_algorithm(W, codebook)
    return codebook, codes, errors

def kmeans_quality(errors, K):
    return np.sum(np.square(errors))+(K/2)*np.log2(errors.shape[0])


if __name__ == "__main__":
    titles = ['data-clean.csv','data-good.csv','data-noisy.csv']
    for t in titles:
        d = data.Data(t)
        reg = linear_regression(d, d.get_headers()[:-1], d.get_headers()[-1])
        s = "File: %s \nSlope X0: %0.3f\nSlope X1: %0.3f\nY-Int: %0.3f\nSum-Squared Error: %0.3f\nR^2: %0.3f\n" \
            % (t, reg[0][0,0], reg[0][1,0], reg[0][2,0], reg[1], reg[2])
        s += "T-Value: " + str(reg[3]) + "\nP-Value: " + str(reg[4][0,:])
        print (s, '\n')
